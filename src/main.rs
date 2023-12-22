use std::ffi::c_void;

use anyhow::{Context, Result};
use esp_idf_sys::{esp, esp_nofail};
use log::info;

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello, world!");

    do_stuff();
}

pub fn do_stuff() {
    init_bt();
    init_ble_mesh().expect("init BLE mesh");
}

pub fn init_bt() {
    // NVS initialisation.
    unsafe {
        if !INITIALIZED {
            let result = esp_idf_sys::nvs_flash_init();
            if result == esp_idf_sys::ESP_ERR_NVS_NO_FREE_PAGES
                || result == esp_idf_sys::ESP_ERR_NVS_NEW_VERSION_FOUND
            {
                ::log::warn!("NVS initialisation failed. Erasing NVS.");
                esp_nofail!(esp_idf_sys::nvs_flash_erase());
                esp_nofail!(esp_idf_sys::nvs_flash_init());
            }

            esp_idf_sys::esp_bt_controller_mem_release(
                esp_idf_sys::esp_bt_mode_t_ESP_BT_MODE_CLASSIC_BT,
            );

            esp_idf_sys::nimble_port_init();

            esp_idf_sys::ble_hs_cfg.sync_cb = Some(on_sync);
            esp_idf_sys::ble_hs_cfg.reset_cb = Some(on_reset);

            // Set initial security capabilities
            esp_idf_sys::ble_hs_cfg.sm_io_cap = esp_idf_sys::BLE_HS_IO_NO_INPUT_OUTPUT as _;
            esp_idf_sys::ble_hs_cfg.set_sm_bonding(0);
            esp_idf_sys::ble_hs_cfg.set_sm_mitm(0);
            esp_idf_sys::ble_hs_cfg.set_sm_sc(1);
            esp_idf_sys::ble_hs_cfg.sm_our_key_dist = 1;
            esp_idf_sys::ble_hs_cfg.sm_their_key_dist = 3;
            esp_idf_sys::ble_hs_cfg.store_status_cb = Some(esp_idf_sys::ble_store_util_status_rr);

            esp_idf_sys::nimble_port_freertos_init(Some(blecent_host_task));
        }

        while !SYNCED {
            esp_idf_sys::vPortYield();
        }

        INITIALIZED = true;
    }
}

fn init_ble_mesh() -> Result<()> {
    unsafe {
        esp!(esp_idf_sys::esp_ble_mesh_register_prov_callback(Some(
            provision_cb
        )))
        .context("register provision callback")?;

        let mut prov = esp_idf_sys::esp_ble_mesh_prov_t {
            uuid: [
                0xdd, 0xdd, 0x92, 0xc4, 0xfd, 0xa1, 0xdf, 0x7c, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                0x0,
            ][..]
                .as_ptr(),
            ..Default::default()
        };

        let mut root_models = [];

        let mut elements = [esp_idf_sys::esp_ble_mesh_elem_t {
            location: 0,

            sig_model_count: root_models.len() as u8,
            sig_models: root_models.as_mut_ptr(),

            vnd_model_count: 0,
            vnd_models: [].as_mut_ptr(),

            ..Default::default()
        }];

        let mut comp = esp_idf_sys::esp_ble_mesh_comp_t {
            cid: 0x02E5,
            elements: elements.as_mut_ptr(),
            element_count: elements.len(),
            ..Default::default()
        };

        info!("esp_ble_mesh_init");
        esp!(esp_idf_sys::esp_ble_mesh_init(&mut prov, &mut comp,)).context("esp ble mesh init")?;

        info!("esp_ble_mesh_node_prov_enable");
        // SHIT HAPPENS HERE
        esp!(esp_idf_sys::esp_ble_mesh_node_prov_enable(
            esp_idf_sys::esp_ble_mesh_prov_bearer_t_ESP_BLE_MESH_PROV_ADV
                | esp_idf_sys::esp_ble_mesh_prov_bearer_t_ESP_BLE_MESH_PROV_GATT
        ))
        .context("esp prov enable")?;

        loop {
            info!("sleeping...");
            std::thread::sleep(std::time::Duration::from_secs(100));
        }
    }
}

unsafe extern "C" fn provision_cb(
    event: esp_idf_sys::esp_ble_mesh_prov_cb_event_t,
    param: *mut esp_idf_sys::esp_ble_mesh_prov_cb_param_t,
) {
    info!("PROVISION EVENT: {}; param: {:?}", event, param);

    match event {
        esp_idf_sys::esp_ble_mesh_prov_cb_event_t_ESP_BLE_MESH_PROV_REGISTER_COMP_EVT => {
            let err_code = (*param).prov_register_comp.err_code;
            log::info!("ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code {}", err_code);
        }
        esp_idf_sys::esp_ble_mesh_prov_cb_event_t_ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT => {
            let err_code = (*param).node_prov_enable_comp.err_code;
            log::info!(
                "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code {}",
                err_code
            );
        }
        esp_idf_sys::esp_ble_mesh_prov_cb_event_t_ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT => {
            let bearer_str = if (*param).node_prov_link_open.bearer
                == esp_idf_sys::esp_ble_mesh_prov_bearer_t_ESP_BLE_MESH_PROV_ADV
            {
                "PB-ADV"
            } else {
                "PB-GATT"
            };
            log::info!(
                "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer {}",
                bearer_str
            );
        }
        esp_idf_sys::esp_ble_mesh_prov_cb_event_t_ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT => {
            let bearer_str = if (*param).node_prov_link_close.bearer
                ==  esp_idf_sys::esp_ble_mesh_prov_bearer_t_ESP_BLE_MESH_PROV_ADV
            {
                "PB-ADV"
            } else {
                "PB-GATT"
            };
            log::info!(
                "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer {}",
                bearer_str
            );
        }
        esp_idf_sys::esp_ble_mesh_prov_cb_event_t_ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT => {
            log::info!("ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        }
        esp_idf_sys::esp_ble_mesh_prov_cb_event_t_ESP_BLE_MESH_NODE_PROV_RESET_EVT => {
            log::info!("ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        }
        esp_idf_sys::esp_ble_mesh_prov_cb_event_t_ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT => {
            let err_code = (*param).node_set_unprov_dev_name_comp.err_code;
            log::info!(
                "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code {}",
                err_code
            );
        }
        _ => {
            log::info!(
                "EVENT: unknown"
            );
        }
    }
}

static mut OWN_ADDR_TYPE: OwnAddrType = OwnAddrType::Public;
static mut INITIALIZED: bool = false;
static mut SYNCED: bool = false;

#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum OwnAddrType {
    Public = esp_idf_sys::BLE_OWN_ADDR_PUBLIC as _,
    Random = esp_idf_sys::BLE_OWN_ADDR_RANDOM as _,
    RpaPublicDefault = esp_idf_sys::BLE_OWN_ADDR_RPA_PUBLIC_DEFAULT as _,
    RpaRandomDefault = esp_idf_sys::BLE_OWN_ADDR_RPA_RANDOM_DEFAULT as _,
}

extern "C" fn on_sync() {
    unsafe {
        esp_idf_sys::ble_hs_util_ensure_addr(0);

        esp_nofail!(esp_idf_sys::ble_hs_id_infer_auto(
            0,
            core::mem::transmute::<&mut OwnAddrType, &mut u8>(&mut OWN_ADDR_TYPE) as *mut _
        ));

        let mut addr = [0; 6];
        esp_nofail!(esp_idf_sys::ble_hs_id_copy_addr(
            OWN_ADDR_TYPE as _,
            addr.as_mut_ptr(),
            core::ptr::null_mut()
        ));
        ::log::info!(
            "Device Address: {:X}:{:X}:{:X}:{:X}:{:X}:{:X}",
            addr[5],
            addr[4],
            addr[3],
            addr[2],
            addr[1],
            addr[0]
        );

        SYNCED = true;
    }
}

extern "C" fn on_reset(reason: i32) {
    ::log::info!("Resetting state; reason={}", reason);
}

extern "C" fn blecent_host_task(_: *mut c_void) {
    unsafe {
        ::log::info!("BLE Host Task Started");
        esp_idf_sys::nimble_port_run();
        esp_idf_sys::nimble_port_freertos_deinit();
    }
}
