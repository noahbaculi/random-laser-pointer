//! Settings persistence in a dedicated flash partition with a debounced save.
//!
//! The record lives in its own `fat`-typed data partition at `0x20000`, outside
//! the first 64 KB that every flash rewrites. One 4 KB sector, rewritten whole.

use embassy_time::{with_timeout, Duration};
use esp_bootloader_esp_idf::partitions::{
    self, DataPartitionSubType, PartitionType, PARTITION_TABLE_MAX_LEN,
};
use esp_hal::peripherals::FLASH;
use esp_storage::FlashStorage;
use random_laser_pointer::record::{self, RECORD_LEN};
use random_laser_pointer::settings::{Settings, Travel};

use crate::control::{with_state, SETTINGS_CHANGED};

// Collapses a slider drag into one sector erase
const SAVE_DEBOUNCE: Duration = Duration::from_secs(2);

pub struct SettingsStore {
    flash: FlashStorage<'static>,
    offset: u32,
}

impl SettingsStore {
    /// Locates the settings partition. `None` if it is missing or the table is unreadable
    pub fn new(flash: FLASH<'static>) -> Option<Self> {
        let mut flash = FlashStorage::new(flash);
        let mut buf = [0u8; PARTITION_TABLE_MAX_LEN];
        let table = partitions::read_partition_table(&mut flash, &mut buf)
            .inspect_err(|e| log::warn!("store: partition table read failed: {e:?}"))
            .ok()?;
        let part = table
            .find_partition(PartitionType::Data(DataPartitionSubType::Fat))
            .inspect_err(|e| log::warn!("store: partition lookup failed: {e:?}"))
            .ok()?;
        let Some(part) = part else {
            log::warn!("store: no settings partition in table");
            return None;
        };
        Some(Self {
            flash,
            offset: part.offset(),
        })
    }

    /// Reads and validates the record. `None` on I/O error or a bad record
    pub fn load(&mut self, travel: Travel) -> Option<Settings> {
        let mut buf = [0u8; RECORD_LEN];
        if let Err(e) = self.flash.read_nor(self.offset, &mut buf) {
            log::warn!("store: read failed: {e:?}");
            return None;
        }
        let settings = record::decode(&buf, travel);
        if settings.is_none() {
            log::warn!("store: no valid record, using defaults");
        }
        settings
    }

    /// Erases the sector and writes the record. Logs and returns on I/O error
    pub fn save(&mut self, settings: &Settings) {
        let start = self.offset;
        if let Err(e) = self.flash.erase(start, start + FlashStorage::SECTOR_SIZE) {
            log::warn!("store: erase failed: {e:?}");
            return;
        }
        if let Err(e) = self.flash.write_nor(start, &record::encode(settings)) {
            log::warn!("store: write failed: {e:?}");
            return;
        }
        log::info!("store: settings saved");
    }
}

#[embassy_executor::task]
pub async fn store_task(mut store: SettingsStore) {
    loop {
        SETTINGS_CHANGED.wait().await;
        while with_timeout(SAVE_DEBOUNCE, SETTINGS_CHANGED.wait())
            .await
            .is_ok()
        {}
        if let Some(settings) = with_state(|s| s.settings) {
            store.save(&settings);
        }
    }
}
