//! Settings persistence in a dedicated flash partition with a debounced save.
//!
//! The record lives in its own `fat`-typed data partition at `0x20000`, outside
//! the first 64 KB that every flash rewrites. Saves alternate between two 4 KB
//! sectors, so a torn write leaves the previous record to load.

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
const SECTOR: u32 = FlashStorage::SECTOR_SIZE;

pub struct SettingsStore {
    flash: FlashStorage<'static>,
    offset: u32,
    // Slot and sequence of the last good record
    slot: u32,
    seq: u32,
}

impl SettingsStore {
    /// Locates the settings partition. `None` if it is missing, under two sectors, or the table is unreadable
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
        if part.len() < 2 * SECTOR {
            log::warn!(
                "store: settings partition is {} bytes, need {}",
                part.len(),
                2 * SECTOR
            );
            return None;
        }
        // Nothing loaded yet, so the first save lands in slot 0
        Some(Self {
            flash,
            offset: part.offset(),
            slot: 1,
            seq: 0,
        })
    }

    /// Returns the newest valid record of the two slots. `None` when neither is readable and valid
    pub fn load(&mut self, travel: Travel) -> Option<Settings> {
        let slots = [0, 1].map(|i| {
            let mut buf = [0u8; RECORD_LEN];
            match self.flash.read_nor(self.offset + i * SECTOR, &mut buf) {
                Ok(()) => record::decode(&buf, travel),
                Err(e) => {
                    log::warn!("store: read slot {i} failed: {e:?}");
                    None
                }
            }
        });
        let Some((slot, seq, settings)) = record::newest(slots) else {
            log::warn!("store: no valid record, using defaults");
            return None;
        };
        self.slot = slot as u32;
        self.seq = seq;
        Some(settings)
    }

    /// Writes the record to the other slot, keeping the last good one intact. Logs and returns on I/O error
    pub fn save(&mut self, settings: &Settings) {
        let slot = 1 - self.slot;
        let seq = self.seq.wrapping_add(1);
        let start = self.offset + slot * SECTOR;
        if let Err(e) = self.flash.erase(start, start + SECTOR) {
            log::warn!("store: erase failed: {e:?}");
            return;
        }
        if let Err(e) = self.flash.write_nor(start, &record::encode(settings, seq)) {
            log::warn!("store: write failed: {e:?}");
            return;
        }
        self.slot = slot;
        self.seq = seq;
        log::info!("store: settings saved to slot {slot}");
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
