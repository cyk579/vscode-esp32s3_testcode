#include "gesture_ble.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <assert.h>
#include <string.h>

static const ble_uuid128_t service_uuid=BLE_UUID128_INIT(GESTURE_UUID_BYTES(1));
static const ble_uuid128_t control_uuid=BLE_UUID128_INIT(GESTURE_UUID_BYTES(2));
static const ble_uuid128_t status_uuid=BLE_UUID128_INIT(GESTURE_UUID_BYTES(3));
static gesture_link_cb link_cb;
static gesture_receive_cb receive_cb;
static gesture_build_cb build_cb;
static gesture_status_cb status_cb;
static esp_err_t (*extra_register)(void);
static void (*extra_tick)(void);
static bool server, subscribed, write_pending, connecting;
static uint16_t conn=BLE_HS_CONN_HANDLE_NONE, control_handle, status_handle;
static uint16_t service_start, service_end;
static uint8_t own_type;
static uint32_t pending_since;
static struct ble_npl_callout timer;
static int gap_event(struct ble_gap_event *e, void *arg);
static void scan(void);
static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time()/1000); }

static int access_control(uint16_t connection, uint16_t attribute, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    (void)attribute; (void)arg;
    if(connection!=conn || ctxt->op!=BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    uint8_t data[GESTURE_FRAME_SIZE]; gesture_frame_t f;
    if(OS_MBUF_PKTLEN(ctxt->om)!=sizeof(data) || os_mbuf_copydata(ctxt->om,0,sizeof(data),data)!=0 || !gesture_decode(data,sizeof(data),&f)) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    receive_cb(&f); return 0;
}
static int access_status(uint16_t connection, uint16_t attribute, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    (void)connection; (void)attribute; (void)arg;
    if(ctxt->op!=BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_READ_NOT_PERMITTED;
    uint8_t bytes[12]={0}; status_cb(bytes);
    return os_mbuf_append(ctxt->om,bytes,sizeof(bytes))==0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}
static const struct ble_gatt_svc_def services[]={
    { .type=BLE_GATT_SVC_TYPE_PRIMARY, .uuid=&service_uuid.u,
      .characteristics=(struct ble_gatt_chr_def[]) {
        { .uuid=&control_uuid.u, .access_cb=access_control, .flags=BLE_GATT_CHR_F_WRITE, .val_handle=&control_handle },
        { .uuid=&status_uuid.u, .access_cb=access_status, .flags=BLE_GATT_CHR_F_READ|BLE_GATT_CHR_F_NOTIFY, .val_handle=&status_handle },
        {0}
      } }, {0}
};
static void advertise(void) {
    struct ble_hs_adv_fields fields={0};
    static const uint8_t manufacturer[]={0xff,0xff,CONFIG_GESTURE_GROUP_ID&255,CONFIG_GESTURE_GROUP_ID>>8};
    fields.flags=BLE_HS_ADV_F_DISC_GEN|BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128=(ble_uuid128_t *)&service_uuid; fields.num_uuids128=1; fields.uuids128_is_complete=1;
    fields.mfg_data=manufacturer; fields.mfg_data_len=sizeof(manufacturer);
    int rc=ble_gap_adv_set_fields(&fields);
    if(rc) { ESP_LOGE("ble","advertise fields: %d",rc); return; }
    struct ble_hs_adv_fields response={0}; const char *name="GestureCar";
    response.name=(uint8_t *)name; response.name_len=strlen(name); response.name_is_complete=1;
    rc=ble_gap_adv_rsp_set_fields(&response);
    if(rc) { ESP_LOGE("ble","scan response: %d",rc); return; }
    struct ble_gap_adv_params params={ .conn_mode=BLE_GAP_CONN_MODE_UND, .disc_mode=BLE_GAP_DISC_MODE_GEN };
    rc=ble_gap_adv_start(own_type,NULL,BLE_HS_FOREVER,&params,gap_event,NULL);
    if(rc) ESP_LOGE("ble","advertise start: %d",rc);
}
static void terminate(void) {
    if(conn!=BLE_HS_CONN_HANDLE_NONE) ble_gap_terminate(conn,BLE_ERR_REM_USER_CONN_TERM);
}
static int on_write(uint16_t connection,const struct ble_gatt_error *error,struct ble_gatt_attr *attr,void *arg) {
    (void)attr; (void)arg;
    if(connection!=conn) return 0;
    write_pending=false;
    if(error->status) terminate();
    return 0;
}
static int on_characteristic(uint16_t connection,const struct ble_gatt_error *error,const struct ble_gatt_chr *chr,void *arg) {
    (void)arg;
    if(connection!=conn) return 0;
    if(error->status==0) {
        if(chr && (chr->properties & BLE_GATT_CHR_PROP_WRITE)) control_handle=chr->val_handle;
    } else if(error->status==BLE_HS_EDONE && control_handle) {
        link_cb(true); ESP_LOGI("ble","Control service ready");
    } else terminate();
    return 0;
}
static int on_service(uint16_t connection,const struct ble_gatt_error *error,const struct ble_gatt_svc *svc,void *arg) {
    (void)arg;
    if(connection!=conn) return 0;
    if(error->status==0 && svc) { service_start=svc->start_handle; service_end=svc->end_handle; }
    else if(error->status==BLE_HS_EDONE && service_start) {
        if(ble_gattc_disc_chrs_by_uuid(conn,service_start,service_end,&control_uuid.u,on_characteristic,NULL)) terminate();
    } else terminate();
    return 0;
}
static void scan(void) {
    if(conn!=BLE_HS_CONN_HANDLE_NONE || connecting || ble_gap_disc_active()) return;
    struct ble_gap_disc_params params={ .passive=1, .filter_duplicates=0 };
    int rc=ble_gap_disc(own_type,10000,&params,gap_event,NULL);
    if(rc) ESP_LOGW("ble","scan: %d",rc);
}
static void tick(struct ble_npl_event *event) {
    (void)event;
    if(server && extra_tick) extra_tick();
    if(conn!=BLE_HS_CONN_HANDLE_NONE) {
        if(server && subscribed) {
            uint8_t status[12]={0}; status_cb(status);
            struct os_mbuf *om=ble_hs_mbuf_from_flat(status,sizeof(status));
            if(om) ble_gatts_notify_custom(conn,status_handle,om);
        } else if(!server && control_handle) {
            if(write_pending) {
                if((uint32_t)(now_ms()-pending_since)>=200) terminate();
            } else {
                gesture_frame_t f={0}; uint8_t bytes[GESTURE_FRAME_SIZE]; build_cb(&f); gesture_encode(&f,bytes);
                write_pending=true; pending_since=now_ms();
                if(ble_gattc_write_flat(conn,control_handle,bytes,sizeof(bytes),on_write,NULL)) { write_pending=false; terminate(); }
            }
        }
    } else if(!server) scan();
    else if(!ble_gap_adv_active()) advertise();
    ble_npl_callout_reset(&timer,ble_npl_time_ms_to_ticks32(server?200:50));
}
static int gap_event(struct ble_gap_event *e,void *arg) {
    (void)arg;
    switch(e->type) {
    case BLE_GAP_EVENT_CONNECT:
        connecting=false;
        if(e->connect.status==0) {
            conn=e->connect.conn_handle; write_pending=false; subscribed=false;
            if(server) link_cb(true);
            else {
                control_handle=service_start=service_end=0;
                if(ble_gattc_disc_svc_by_uuid(conn,&service_uuid.u,on_service,NULL)) terminate();
            }
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        conn=BLE_HS_CONN_HANDLE_NONE; subscribed=false; write_pending=false; connecting=false;
        if(!server) control_handle=0;
        link_cb(false); break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if(e->subscribe.attr_handle==status_handle) subscribed=e->subscribe.cur_notify;
        break;
    case BLE_GAP_EVENT_DISC: {
        if(server || conn!=BLE_HS_CONN_HANDLE_NONE || connecting) break;
        struct ble_hs_adv_fields f;
        if(ble_hs_adv_parse_fields(&f,e->disc.data,e->disc.length_data)) break;
        if(f.mfg_data_len!=4 || f.mfg_data[0]!=255 || f.mfg_data[1]!=255 ||
            (f.mfg_data[2]|(f.mfg_data[3]<<8))!=CONFIG_GESTURE_GROUP_ID) break;
        bool match=false;
        for(int i=0;i<f.num_uuids128;++i) if(!ble_uuid_cmp(&f.uuids128[i].u,&service_uuid.u)) match=true;
        if(!match || e->disc.event_type!=BLE_HCI_ADV_RPT_EVTYPE_ADV_IND) break;
        if(ble_gap_disc_cancel()) break;
        connecting=true;
        if(ble_gap_connect(own_type,&e->disc.addr,5000,NULL,gap_event,NULL)) connecting=false;
        break;
    }
    default: break;
    }
    return 0;
}
static void on_reset(int reason) {
    ESP_LOGE("ble","host reset %d",reason); conn=BLE_HS_CONN_HANDLE_NONE; connecting=false;
    ble_npl_callout_stop(&timer); link_cb(false);
}
static void on_sync(void) {
    if(ble_hs_util_ensure_addr(0) || ble_hs_id_infer_auto(0,&own_type)) return;
    if(server) advertise(); else scan();
    ble_npl_callout_reset(&timer,ble_npl_time_ms_to_ticks32(50));
}
static void host_task(void *arg) { (void)arg; nimble_port_run(); nimble_port_freertos_deinit(); }
static esp_err_t start(void) {
    esp_err_t err=nvs_flash_init();
    if(err==ESP_ERR_NVS_NO_FREE_PAGES || err==ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err=nvs_flash_erase(); if(err!=ESP_OK) return err; err=nvs_flash_init();
    }
    if(err!=ESP_OK) return err;
    err=nimble_port_init(); if(err!=ESP_OK) return err;
    ble_hs_cfg.sync_cb=on_sync; ble_hs_cfg.reset_cb=on_reset;
    ble_svc_gap_init(); ble_svc_gatt_init();
    ble_svc_gap_device_name_set(server?"GestureCar":"GestureRemote");
    if(server && (ble_gatts_count_cfg(services) || ble_gatts_add_svcs(services))) return ESP_FAIL;
    if(server && extra_register && (err=extra_register())!=ESP_OK) return err;
    ble_npl_callout_init(&timer,nimble_port_get_dflt_eventq(),tick,NULL);
    nimble_port_freertos_init(host_task); return ESP_OK;
}
esp_err_t gesture_ble_server_start(gesture_link_cb link,gesture_receive_cb receive,gesture_status_cb status) {
    return gesture_ble_server_start_extended(link,receive,status,NULL,NULL);
}
esp_err_t gesture_ble_server_start_extended(gesture_link_cb link,gesture_receive_cb receive,
    gesture_status_cb status,esp_err_t (*register_services)(void),void (*tick_services)(void)) {
    server=true; link_cb=link; receive_cb=receive; status_cb=status;
    extra_register=register_services; extra_tick=tick_services; return start();
}
esp_err_t gesture_ble_client_start(gesture_link_cb link,gesture_build_cb build) {
    server=false; link_cb=link; build_cb=build; return start();
}
