#include "music_player.h"
#include "host/ble_hs.h"
static const ble_uuid128_t svc=BLE_UUID128_INIT(MEDIA_UUID_BYTES(1));
static const ble_uuid128_t cmd=BLE_UUID128_INIT(MEDIA_UUID_BYTES(2));
static const ble_uuid128_t state=BLE_UUID128_INIT(MEDIA_UUID_BYTES(3));
static uint16_t state_handle;
static int access_command(uint16_t conn,uint16_t attr,struct ble_gatt_access_ctxt *ctx,void *arg) {
    (void)conn; (void)attr; (void)arg;
    uint8_t bytes[MEDIA_COMMAND_BYTES]; media_command_t command;
    if(ctx->op!=BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    if(OS_MBUF_PKTLEN(ctx->om)!=sizeof(bytes) || os_mbuf_copydata(ctx->om,0,sizeof(bytes),bytes) ||
       !media_decode(bytes,sizeof(bytes),&command)) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    return music_player_submit(&command) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}
static int access_state(uint16_t conn,uint16_t attr,struct ble_gatt_access_ctxt *ctx,void *arg) {
    (void)conn; (void)attr; (void)arg;
    if(ctx->op!=BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_READ_NOT_PERMITTED;
    uint8_t bytes[MEDIA_STATUS_BYTES]; music_player_status(bytes);
    return os_mbuf_append(ctx->om,bytes,sizeof(bytes))==0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}
static const struct ble_gatt_svc_def services[]={
    {.type=BLE_GATT_SVC_TYPE_PRIMARY,.uuid=&svc.u,.characteristics=(struct ble_gatt_chr_def[]){
        {.uuid=&cmd.u,.access_cb=access_command,.flags=BLE_GATT_CHR_F_WRITE},
        {.uuid=&state.u,.access_cb=access_state,.flags=BLE_GATT_CHR_F_READ|BLE_GATT_CHR_F_NOTIFY,.val_handle=&state_handle},
        {0}}}, {0}
};
esp_err_t music_gatt_register(void) {
    return ble_gatts_count_cfg(services) || ble_gatts_add_svcs(services) ? ESP_FAIL : ESP_OK;
}
void music_gatt_tick(void) { if(state_handle) ble_gatts_chr_updated(state_handle); }
