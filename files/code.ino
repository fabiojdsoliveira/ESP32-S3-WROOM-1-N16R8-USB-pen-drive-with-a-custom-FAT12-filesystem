/*
 * =============================================================================
 * ESP32-S3 USB Mass Storage + FAT12 File System Library
 * =============================================================================
 *
 * Hardware : ESP32-S3-WROOM-1-N16R8 (16MB Flash, 8MB PSRAM)
 *
 * -----------------------------------------------------------------------------
 * ARDUINO IDE — TOOLS MENU SETTINGS
 * -----------------------------------------------------------------------------
 *   Board                   : ESP32S3 Dev Module
 *   USB CDC On Boot         : Disabled
 *   CPU Frequency           : 240MHz (WiFi)
 *   Core Debug Level        : None
 *   USB DFU On Boot         : Disabled
 *   Events Run On           : Core 1
 *   Flash Mode              : QIO 80MHz
 *   Flash Size              : 16MB (128Mb)
 *   JTAG Adapter            : Disabled
 *   Arduino Runs On         : Core 1
 *   USB Firmware MSC On Boot: Disabled
 *   Partition Scheme        : Custom   ← requires partitions.csv in sketch folder
 *   PSRAM                   : OPI PSRAM
 *   Upload Mode             : UART0 / Hardware CDC
 *   Upload Speed            : 921600
 *   USB Mode                : USB-OTG (TinyUSB)
 *   Zigbee Mode             : Disabled
 *
 * -----------------------------------------------------------------------------
 * PROGRAMMING
 * -----------------------------------------------------------------------------
 *   The board is programmed via the USB port connected to the USB-to-Serial
 *   converter (UART0), NOT via the native USB port. Make sure to select the
 *   correct COM port in the Arduino IDE corresponding to the serial converter.
 *
 * -----------------------------------------------------------------------------
 * REQUIRED LIBRARIES
 * -----------------------------------------------------------------------------
 *   - ArduinoJson 7.4.3 (Benoit Blanchon) — install via Library Manager
 *
 *   The following headers from the Arduino core are used (no install needed):
 *     - USB.h           (TinyUSB Arduino wrapper)
 *     - USBMSC.h        (Mass Storage Class)
 *     - esp_partition.h (raw flash partition access)
 *
 * -----------------------------------------------------------------------------
 * OVERVIEW
 * -----------------------------------------------------------------------------
 *   This firmware exposes an internal FAT12 flash partition as a USB Mass
 *   Storage device (pen drive). When connected to a PC, the partition appears
 *   as a removable drive where the user can create, edit and delete files
 *   normally. When disconnected, the ESP32 reads and writes those files
 *   directly from flash using a custom FAT12 implementation.
 *
 *   The key challenge solved here is compatibility: the ESP-IDF FFat library
 *   uses a wear-levelling layer that makes its on-flash layout incompatible
 *   with what Windows writes via USB MSC. This implementation bypasses that
 *   layer entirely, reading and writing raw flash sectors, which means both
 *   the PC and the ESP32 see exactly the same data.
 *
 * -----------------------------------------------------------------------------
 * ARCHITECTURE
 * -----------------------------------------------------------------------------
 *
 *   PC (Windows)                     ESP32-S3
 *   ─────────────────────────        ────────────────────────────────────────
 *   USB MSC (pen drive)    ◄────►   onRead()  → esp_partition_read()
 *                                   onWrite() → read-modify-write (4096 block)
 *
 *   When USB is active:
 *     - PC has exclusive access to the FAT partition
 *     - ESP32 does not touch the partition
 *
 *   When USB is disconnected/ejected:
 *     - ESP32 reads the FAT12 filesystem by parsing raw sectors
 *     - Files can be listed, read and written by the firmware
 *
 * -----------------------------------------------------------------------------
 * FAT12 IMPLEMENTATION
 * -----------------------------------------------------------------------------
 *   - Boot sector parsing (bytes per sector, sectors per cluster, FAT layout)
 *   - Full FAT table loaded into RAM for fast cluster chain traversal
 *   - Long File Name (LFN) read support: UTF-16 → UTF-8 (incl. Portuguese)
 *   - Long File Name (LFN) write support: UTF-8 → UTF-16, auto 8.3 name gen
 *   - Recursive directory listing with indentation
 *   - Path navigation: "/folder/subfolder/file.txt"
 *
 * -----------------------------------------------------------------------------
 * ⚠️  IMPORTANT: NO WEAR LEVELLING
 * -----------------------------------------------------------------------------
 *   This implementation writes directly to flash without any wear levelling.
 *   Flash memory has a limited write endurance — typically ~100,000 erase
 *   cycles per 4096-byte block. If the same blocks are written repeatedly
 *   (e.g. appending sensor data every minute), those blocks will eventually
 *   degrade and fail.
 *
 *   For occasional writes (copying music files, updating config) this is not
 *   a concern in practice. For frequent writes (data logging, counters):
 *     - Use PSRAM as a write buffer and flush to flash periodically
 *     - Implement a simple wear-levelling rotation across multiple blocks
 *     - Use an external SD card (FAT32, no write limit concern)
 *
 * -----------------------------------------------------------------------------
 * PUBLIC API
 * -----------------------------------------------------------------------------
 *   listDirectory("/", true)                      list all files recursively
 *   listDirectory("/FolderName", false)           list one folder only
 *   fatReadFileToBuffer("/file.json", &size)      read file into RAM buffer
 *   fatWriteFile("/file.json", data, size)        create or overwrite file
 *   fatAppendFile("/log.txt", data, size)         append to file (create if absent)
 *
 * =============================================================================
 */

#include "USB.h"
#include "USBMSC.h"
#include "esp_partition.h"
#include <ArduinoJson.h>

#define DEBUG true   // set to false to suppress diagnostic Serial output

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------
#define PARTITION_LABEL    "storage"       // must match partitions.csv
#define READ_INTERVAL_MS   10000           // how often to run the main loop task
#define DISK_SECTOR_SIZE   512             // standard USB MSC sector size
#define FLASH_ERASE_SIZE   4096            // minimum erase unit of SPI flash
#define DISK_SECTOR_COUNT  (0x800000 / DISK_SECTOR_SIZE)  // 8MB → 16384 sectors
#define MAX_LFN_LEN        256             // maximum long filename length (chars)
#define PSRAM_THRESHOLD    51200           // files larger than 50KB use PSRAM

// File paths
#define CONFIG_FILE        "/config.json"
#define DATA_FILE          "/data.json"

// Default config values — used when config.json does not exist
#define DEFAULT_SSID       "MyNetwork"
#define DEFAULT_PASSWORD   "MyPassword"

// =============================================================================
// STRUCTS AND TYPES
// =============================================================================

typedef struct __attribute__((packed)) {
  uint8_t  seq;
  uint16_t name0[5];
  uint8_t  attr;
  uint8_t  type;
  uint8_t  checksum;
  uint16_t name1[6];
  uint16_t first_cluster;
  uint16_t name2[2];
} fat_lfn_entry_t;

typedef struct __attribute__((packed)) {
  char     name[8];
  char     ext[3];
  uint8_t  attr;
  uint8_t  reserved[8];
  uint16_t first_cluster_hi;
  uint16_t write_time;
  uint16_t write_date;
  uint16_t first_cluster_lo;
  uint32_t file_size;
} fat_dir_entry_t;

typedef struct {
  char     longName[MAX_LFN_LEN];
  uint16_t first_cluster;
  uint32_t file_size;
  bool     is_dir;
  uint32_t dir_sector;
  uint32_t dir_entry_idx;
} FatEntry;

typedef void (*FatEntryCallback)(FatEntry* entry, void* ctx);

typedef struct {
  const char* target;
  FatEntry    result;
  bool        found;
} FindCtx;

typedef struct {
  int  indent;
  bool recursive;
} ListCtx;

// =============================================================================
// GLOBAL STATE
// =============================================================================

USBMSC msc;
volatile bool usbMounted     = false;
bool          usbMountedPrev = false;
unsigned long lastReadTime   = 0;
static uint8_t rwBlock[FLASH_ERASE_SIZE];

static struct {
  uint16_t bytes_per_sector;
  uint8_t  sectors_per_cluster;
  uint32_t fat_start;
  uint32_t root_start;
  uint32_t data_start;
  uint8_t  num_fats;
  uint16_t fat_size_sectors;
  uint16_t root_entry_count;
  bool     valid;
} fatInfo;

static uint8_t* fatTable     = nullptr;
static uint32_t fatTableSize = 0;

// =============================================================================
// FLASH ACCESS
// =============================================================================

static const esp_partition_t* getPartition() {
  static const esp_partition_t* part = nullptr;
  if (!part) {
    part = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_LABEL
    );
    if (!part) Serial.println("[ERROR] Partition '" PARTITION_LABEL "' not found");
  }
  return part;
}

static bool readSector(uint32_t lba, uint8_t* buf) {
  const esp_partition_t* part = getPartition();
  if (!part) return false;
  if (esp_partition_read(part, lba * DISK_SECTOR_SIZE, buf, DISK_SECTOR_SIZE) != ESP_OK) {
    if (DEBUG) Serial.printf("[ERROR] readSector failed at LBA %lu\n", lba);
    return false;
  }
  return true;
}

static bool writeSector(uint32_t lba, const uint8_t* buf) {
  const esp_partition_t* part = getPartition();
  if (!part) return false;
  uint32_t addr      = lba * DISK_SECTOR_SIZE;
  uint32_t eraseAddr = (addr / FLASH_ERASE_SIZE) * FLASH_ERASE_SIZE;
  if (esp_partition_read(part, eraseAddr, rwBlock, FLASH_ERASE_SIZE) != ESP_OK) {
    if (DEBUG) Serial.printf("[ERROR] writeSector read failed at LBA %lu\n", lba);
    return false;
  }
  memcpy(rwBlock + (addr - eraseAddr), buf, DISK_SECTOR_SIZE);
  if (esp_partition_erase_range(part, eraseAddr, FLASH_ERASE_SIZE) != ESP_OK) {
    if (DEBUG) Serial.printf("[ERROR] writeSector erase failed at LBA %lu\n", lba);
    return false;
  }
  if (esp_partition_write(part, eraseAddr, rwBlock, FLASH_ERASE_SIZE) != ESP_OK) {
    if (DEBUG) Serial.printf("[ERROR] writeSector write failed at LBA %lu\n", lba);
    return false;
  }
  return true;
}

static bool writeToFlash(const esp_partition_t* part, uint32_t addr,
                          const uint8_t* buffer, uint32_t bufsize) {
  uint32_t remaining = bufsize, srcOffset = 0;
  while (remaining > 0) {
    uint32_t eraseAddr     = (addr / FLASH_ERASE_SIZE) * FLASH_ERASE_SIZE;
    uint32_t offsetInBlock = addr - eraseAddr;
    uint32_t canWrite      = FLASH_ERASE_SIZE - offsetInBlock;
    if (canWrite > remaining) canWrite = remaining;
    if (esp_partition_read(part, eraseAddr, rwBlock, FLASH_ERASE_SIZE) != ESP_OK) return false;
    memcpy(rwBlock + offsetInBlock, buffer + srcOffset, canWrite);
    if (esp_partition_erase_range(part, eraseAddr, FLASH_ERASE_SIZE) != ESP_OK) return false;
    if (esp_partition_write(part, eraseAddr, rwBlock, FLASH_ERASE_SIZE) != ESP_OK) return false;
    addr += canWrite; srcOffset += canWrite; remaining -= canWrite;
  }
  return true;
}

// =============================================================================
// USB MSC CALLBACKS
// =============================================================================

static int32_t onRead(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
  const esp_partition_t* part = getPartition();
  if (!part) return -1;
  if (esp_partition_read(part, (lba * DISK_SECTOR_SIZE) + offset, buffer, bufsize) != ESP_OK) return -1;
  return bufsize;
}

static int32_t onWrite(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
  const esp_partition_t* part = getPartition();
  if (!part) return -1;
  if (!writeToFlash(part, (lba * DISK_SECTOR_SIZE) + offset, buffer, bufsize)) return -1;
  return bufsize;
}

static bool onStartStop(uint8_t power_condition, bool start, bool load_eject) {
  (void)power_condition; (void)start; (void)load_eject; return true;
}

// =============================================================================
// USB EVENT CALLBACK
// =============================================================================

static void usbEventCallback(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data) {
  if (event_base == ARDUINO_USB_EVENTS) {
    switch (event_id) {
      case ARDUINO_USB_STARTED_EVENT:
        usbMounted = true;
        if (fatTable) { free(fatTable); fatTable = nullptr; }
        fatInfo.valid = false;
        Serial.println("[USB] Connected to PC - MSC mode active");
        break;
      case ARDUINO_USB_STOPPED_EVENT:
        usbMounted = false;
        Serial.println("[USB] Disconnected from PC");
        lastReadTime = 0;
        break;
      case ARDUINO_USB_SUSPEND_EVENT:
        usbMounted = false;
        Serial.println("[USB] Suspended / Ejected");
        lastReadTime = 0;
        break;
      case ARDUINO_USB_RESUME_EVENT:
        usbMounted = true;
        Serial.println("[USB] Resumed");
        break;
      default: break;
    }
  }
}

// =============================================================================
// UTF-16 ↔ UTF-8
// =============================================================================

static int utf16ToUtf8(uint16_t cp, char* dst) {
  if (cp == 0xFFFF) return 0;
  if (cp < 0x80)   { dst[0] = (char)cp; return 1; }
  if (cp < 0x800)  { dst[0] = (char)(0xC0 | (cp >> 6)); dst[1] = (char)(0x80 | (cp & 0x3F)); return 2; }
  dst[0] = (char)(0xE0 | (cp >> 12));
  dst[1] = (char)(0x80 | ((cp >> 6) & 0x3F));
  dst[2] = (char)(0x80 | (cp & 0x3F));
  return 3;
}

static void utf16bufToStr(uint16_t* utf16buf, int len, char* out, int outSize) {
  int outIdx = 0;
  for (int i = 0; i < len && outIdx < outSize - 4; i++) {
    uint16_t cp = utf16buf[i];
    if (cp == 0x0000 || cp == 0xFFFF) break;
    char tmp[4]; int bytes = utf16ToUtf8(cp, tmp);
    for (int b = 0; b < bytes && outIdx < outSize - 1; b++) out[outIdx++] = tmp[b];
  }
  out[outIdx] = '\0';
}

static int utf8ToUtf16(const char* src, uint16_t* dst, int maxLen) {
  int outIdx = 0;
  const uint8_t* p = (const uint8_t*)src;
  while (*p && outIdx < maxLen) {
    uint16_t cp;
    if      (*p < 0x80)            { cp = *p++; }
    else if ((*p & 0xE0) == 0xC0) { cp = (*p++ & 0x1F) << 6; cp |= (*p++ & 0x3F); }
    else if ((*p & 0xF0) == 0xE0) { cp = (*p++ & 0x0F) << 12; cp |= (*p++ & 0x3F) << 6; cp |= (*p++ & 0x3F); }
    else                           { p++; cp = '?'; }
    dst[outIdx++] = cp;
  }
  return outIdx;
}

// =============================================================================
// LFN HELPERS
// =============================================================================

static void lfnExtractChars(fat_lfn_entry_t* lfn, uint16_t* utf16buf, int pos) {
  const uint16_t* fields[3] = { lfn->name0, lfn->name1, lfn->name2 };
  const int sizes[3] = { 5, 6, 2 };
  int idx = pos;
  for (int f = 0; f < 3; f++)
    for (int c = 0; c < sizes[f]; c++)
      if (idx < MAX_LFN_LEN) utf16buf[idx++] = fields[f][c];
}

static uint8_t lfnChecksum(const char* name83) {
  uint8_t sum = 0;
  for (int i = 11; i; i--) sum = ((sum & 1) << 7) + (sum >> 1) + (uint8_t)*name83++;
  return sum;
}

static void lfnFillEntry(fat_lfn_entry_t* lfn, uint8_t seq, bool isLast,
                          const uint16_t* utf16, int totalChars, uint8_t checksum) {
  memset(lfn, 0xFF, sizeof(fat_lfn_entry_t));
  lfn->seq           = seq | (isLast ? 0x40 : 0x00);
  lfn->attr          = 0x0F;
  lfn->type          = 0x00;
  lfn->checksum      = checksum;
  lfn->first_cluster = 0x0000;
  uint16_t* fields[3] = { lfn->name0, lfn->name1, lfn->name2 };
  const int sizes[3]  = { 5, 6, 2 };
  int charIdx = (seq - 1) * 13;
  for (int f = 0; f < 3; f++)
    for (int c = 0; c < sizes[f]; c++) {
      if      (charIdx < totalChars)  fields[f][c] = utf16[charIdx++];
      else if (charIdx == totalChars) { fields[f][c] = 0x0000; charIdx++; }
    }
}

// =============================================================================
// FAT12 — BOOT SECTOR AND TABLE
// =============================================================================

static bool fatReadBootSector() {
  if (fatInfo.valid) return true;
  uint8_t buf[DISK_SECTOR_SIZE];
  if (!readSector(0, buf)) { Serial.println("[ERROR] Failed to read boot sector"); return false; }
  if (buf[510] != 0x55 || buf[511] != 0xAA) {
    Serial.println("[ERROR] Partition not formatted - please format the drive on the PC first");
    return false;
  }
  fatInfo.bytes_per_sector    = buf[11] | (buf[12] << 8);
  fatInfo.sectors_per_cluster = buf[13];
  uint16_t reserved           = buf[14] | (buf[15] << 8);
  fatInfo.num_fats            = buf[16];
  fatInfo.root_entry_count    = buf[17] | (buf[18] << 8);
  fatInfo.fat_size_sectors    = buf[22] | (buf[23] << 8);
  fatInfo.fat_start           = reserved;
  fatInfo.root_start          = fatInfo.fat_start + (fatInfo.num_fats * fatInfo.fat_size_sectors);
  fatInfo.data_start          = fatInfo.root_start + ((fatInfo.root_entry_count * 32) / fatInfo.bytes_per_sector);
  fatInfo.valid               = true;
  if (DEBUG) Serial.printf("[FAT12] fat_start=%lu root_start=%lu data_start=%lu spc=%d\n",
                            fatInfo.fat_start, fatInfo.root_start, fatInfo.data_start, fatInfo.sectors_per_cluster);
  return true;
}

static bool fatLoadTable() {
  if (fatTable) return true;
  if (!fatInfo.valid) return false;
  fatTableSize = fatInfo.fat_size_sectors * DISK_SECTOR_SIZE;
  fatTable = (uint8_t*)malloc(fatTableSize);
  if (!fatTable) { Serial.println("[ERROR] Out of memory loading FAT table"); return false; }
  for (uint16_t s = 0; s < fatInfo.fat_size_sectors; s++) {
    if (!readSector(fatInfo.fat_start + s, fatTable + (s * DISK_SECTOR_SIZE))) {
      Serial.printf("[ERROR] Failed to read FAT sector %u\n", s);
      free(fatTable); fatTable = nullptr; return false;
    }
  }
  if (DEBUG) Serial.printf("[FAT12] FAT table loaded (%lu bytes)\n", fatTableSize);
  return true;
}

static uint16_t fat12GetEntry(uint16_t cluster) {
  if (!fatTable) return 0xFFF;
  uint32_t bo = (cluster * 3) / 2;
  if (bo + 1 >= fatTableSize) return 0xFFF;
  uint16_t val = fatTable[bo] | (fatTable[bo + 1] << 8);
  return (cluster & 1) ? (val >> 4) : (val & 0x0FFF);
}

static void fat12SetEntry(uint16_t cluster, uint16_t value) {
  if (!fatTable) return;
  uint32_t bo = (cluster * 3) / 2;
  if (bo + 1 >= fatTableSize) return;
  if (cluster & 1) {
    fatTable[bo]     = (fatTable[bo] & 0x0F) | ((value & 0x0F) << 4);
    fatTable[bo + 1] = (value >> 4) & 0xFF;
  } else {
    fatTable[bo]     = value & 0xFF;
    fatTable[bo + 1] = (fatTable[bo + 1] & 0xF0) | ((value >> 8) & 0x0F);
  }
}

static bool fatFlushTable() {
  for (uint8_t f = 0; f < fatInfo.num_fats; f++) {
    uint32_t fatStart = fatInfo.fat_start + f * fatInfo.fat_size_sectors;
    for (uint16_t s = 0; s < fatInfo.fat_size_sectors; s++) {
      if (!writeSector(fatStart + s, fatTable + s * DISK_SECTOR_SIZE)) {
        Serial.printf("[ERROR] Failed to flush FAT copy %u sector %u\n", f, s);
        return false;
      }
    }
  }
  return true;
}

static uint32_t clusterToSector(uint16_t cluster) {
  return fatInfo.data_start + (cluster - 2) * fatInfo.sectors_per_cluster;
}

static uint16_t fatFindFreeCluster() {
  uint16_t totalClusters = (fatInfo.fat_size_sectors * DISK_SECTOR_SIZE * 2) / 3;
  for (uint16_t i = 2; i < totalClusters; i++) if (fat12GetEntry(i) == 0x000) return i;
  return 0xFFFF;
}

static void fatFreeChain(uint16_t firstCluster) {
  uint16_t cluster = firstCluster;
  while (cluster >= 0x002 && cluster <= 0xFEF) {
    uint16_t next = fat12GetEntry(cluster);
    fat12SetEntry(cluster, 0x000);
    cluster = next;
  }
}

// =============================================================================
// FAT12 — DIRECTORY ITERATION
// =============================================================================

static void fatIterateDir(uint16_t cluster, bool is_root, FatEntryCallback cb, void* ctx) {
  uint16_t utf16buf[MAX_LFN_LEN];
  memset(utf16buf, 0xFF, sizeof(utf16buf));
  bool hasLfn = false;
  uint32_t sector, maxSectors;
  uint16_t currentCluster = cluster;
  if (is_root) { sector = fatInfo.root_start; maxSectors = (fatInfo.root_entry_count * 32) / fatInfo.bytes_per_sector; }
  else         { sector = clusterToSector(cluster); maxSectors = fatInfo.sectors_per_cluster; }
  uint32_t sectorInCluster = 0;
  while (true) {
    uint8_t localBuf[DISK_SECTOR_SIZE];
    if (!readSector(sector, localBuf)) break;
    fat_dir_entry_t* entries = (fat_dir_entry_t*)localBuf;
    uint32_t eps = DISK_SECTOR_SIZE / sizeof(fat_dir_entry_t);
    for (uint32_t i = 0; i < eps; i++) {
      fat_dir_entry_t* e = &entries[i];
      if (e->name[0] == 0x00) return;
      if ((uint8_t)e->name[0] == 0xE5) { hasLfn = false; memset(utf16buf, 0xFF, sizeof(utf16buf)); continue; }
      if (e->attr == 0x0F) {
        fat_lfn_entry_t* lfn = (fat_lfn_entry_t*)e;
        uint8_t seq = lfn->seq & 0x1F;
        if (seq == 0 || seq > 20) continue;
        lfnExtractChars(lfn, utf16buf, (seq - 1) * 13);
        hasLfn = true; continue;
      }
      if (e->attr & 0x08) { hasLfn = false; memset(utf16buf, 0xFF, sizeof(utf16buf)); continue; }
      FatEntry entry;
      entry.first_cluster = e->first_cluster_lo;
      entry.file_size     = e->file_size;
      entry.is_dir        = (e->attr & 0x10) != 0;
      entry.dir_sector    = sector;
      entry.dir_entry_idx = i;
      if (hasLfn) {
        utf16bufToStr(utf16buf, MAX_LFN_LEN, entry.longName, sizeof(entry.longName));
      } else {
        char name[9] = {0}, ext[4] = {0};
        memcpy(name, e->name, 8); memcpy(ext, e->ext, 3);
        for (int k = 7; k >= 0 && name[k] == ' '; k--) name[k] = 0;
        for (int k = 2; k >= 0 && ext[k]  == ' '; k--) ext[k]  = 0;
        if (strlen(ext) > 0) snprintf(entry.longName, sizeof(entry.longName), "%s.%s", name, ext);
        else                 snprintf(entry.longName, sizeof(entry.longName), "%s", name);
      }
      cb(&entry, ctx);
      hasLfn = false; memset(utf16buf, 0xFF, sizeof(utf16buf));
    }
    sector++; sectorInCluster++;
    if (is_root) {
      if (sectorInCluster >= maxSectors) break;
    } else {
      if (sectorInCluster >= fatInfo.sectors_per_cluster) {
        uint16_t next = fat12GetEntry(currentCluster);
        if (next >= 0xFF8) break;
        currentCluster = next;
        sector = clusterToSector(currentCluster);
        sectorInCluster = 0;
      }
    }
  }
}

// =============================================================================
// FAT12 — PATH NAVIGATION
// =============================================================================

static void findCallback(FatEntry* e, void* c) {
  FindCtx* ctx = (FindCtx*)c;
  if (!ctx->found && strcasecmp(e->longName, ctx->target) == 0) { ctx->result = *e; ctx->found = true; }
}

static bool fatFindInDir(uint16_t cluster, bool is_root, const char* name, FatEntry* out) {
  FindCtx ctx; ctx.target = name; ctx.found = false;
  fatIterateDir(cluster, is_root, findCallback, &ctx);
  if (ctx.found) *out = ctx.result;
  return ctx.found;
}

static bool fatResolvePath(const char* path, FatEntry* out) {
  char buf[512]; strncpy(buf, path, sizeof(buf) - 1); buf[sizeof(buf)-1] = '\0';
  char* p = buf; if (*p == '/') p++; if (*p == '\0') return false;
  uint16_t curCluster = 0; bool curIsRoot = true;
  char* token = strtok(p, "/");
  while (token != nullptr) {
    FatEntry entry;
    if (!fatFindInDir(curCluster, curIsRoot, token, &entry)) return false;
    char* next = strtok(nullptr, "/");
    if (next == nullptr) { *out = entry; return true; }
    if (!entry.is_dir) return false;
    curCluster = entry.first_cluster; curIsRoot = false; token = next;
  }
  return false;
}

// =============================================================================
// FAT12 — 8.3 NAME GENERATION
// =============================================================================

static void fatGenerate83(const char* longName, char* out83) {
  memset(out83, ' ', 11); out83[11] = 0;
  const char* dot = strrchr(longName, '.');
  char baseName[256] = {0}, ext[4] = {0};
  if (dot) {
    int extLen = strlen(dot + 1); if (extLen > 3) extLen = 3;
    for (int i = 0; i < extLen; i++) ext[i] = toupper((unsigned char)dot[1 + i]);
    int baseLen = dot - longName; if (baseLen > 255) baseLen = 255;
    strncpy(baseName, longName, baseLen);
  } else { strncpy(baseName, longName, 255); }
  char clean[9] = {0}; int ci = 0;
  for (int i = 0; baseName[i] && ci < 8; i++) {
    unsigned char c = (unsigned char)baseName[i];
    if (c < 0x80 && c != ' ' && c != '.' && c != ',' && c != '[' && c != ']') clean[ci++] = toupper(c);
  }
  if (ci == 0) { strcpy(clean, "FILE"); ci = 4; }
  for (int n = 0; n <= 9; n++) {
    char candidate[12] = {0};
    if (n == 0) { if (ci <= 8) snprintf(candidate, sizeof(candidate), "%-8s", clean); else continue; }
    else {
      char suffix[4]; snprintf(suffix, sizeof(suffix), "~%d", n);
      int baseChars = 8 - strlen(suffix); if (baseChars < 1) baseChars = 1;
      snprintf(candidate, sizeof(candidate), "%-*.*s%s", baseChars, baseChars, clean, suffix);
    }
    char full83[12] = {0};
    memcpy(full83, candidate, 8); memcpy(full83 + 8, ext, 3); full83[11] = 0;
    for (int i = 0; i < 8; i++) if (candidate[i] == 0) candidate[i] = ' ';
    bool exists = false;
    uint32_t root_sectors = (fatInfo.root_entry_count * 32) / fatInfo.bytes_per_sector;
    for (uint32_t s = 0; s < root_sectors && !exists; s++) {
      uint8_t buf[DISK_SECTOR_SIZE];
      if (!readSector(fatInfo.root_start + s, buf)) break;
      fat_dir_entry_t* entries = (fat_dir_entry_t*)buf;
      uint32_t eps = DISK_SECTOR_SIZE / sizeof(fat_dir_entry_t);
      for (uint32_t i = 0; i < eps && !exists; i++) {
        if (entries[i].name[0] == 0x00) { s = root_sectors; break; }
        if ((uint8_t)entries[i].name[0] == 0xE5) continue;
        if (entries[i].attr == 0x0F || entries[i].attr & 0x08) continue;
        char entryName[12] = {0};
        memcpy(entryName, entries[i].name, 8); memcpy(entryName + 8, entries[i].ext, 3);
        if (memcmp(entryName, full83, 11) == 0) exists = true;
      }
    }
    if (!exists) { memcpy(out83, full83, 11); return; }
  }
  char fb[12] = {0}; snprintf(fb, 12, "FILE%07lu", millis() % 9999999);
  memcpy(out83, fb, 11);
}

// =============================================================================
// FAT12 — WRITE SUPPORT
// =============================================================================

static bool fatUpdateDirEntry(uint32_t dir_sector, uint32_t dir_entry_idx,
                               uint16_t first_cluster, uint32_t file_size) {
  uint8_t buf[DISK_SECTOR_SIZE];
  if (!readSector(dir_sector, buf)) return false;
  fat_dir_entry_t* e = &((fat_dir_entry_t*)buf)[dir_entry_idx];
  e->first_cluster_lo = first_cluster;
  e->file_size        = file_size;
  return writeSector(dir_sector, buf);
}

static bool fatCreateEntryWithLFN(const char* longName, uint16_t first_cluster,
                                   uint32_t file_size,
                                   uint32_t* out_sector, uint32_t* out_idx) {
  uint16_t utf16[MAX_LFN_LEN];
  int nameLen      = utf8ToUtf16(longName, utf16, MAX_LFN_LEN);
  int lfnCount     = (nameLen + 12) / 13;
  int totalEntries = lfnCount + 1;
  char name83[12]; fatGenerate83(longName, name83);
  uint8_t checksum = lfnChecksum(name83);
  if (DEBUG) Serial.printf("[FAT] Generated 8.3 name: '%.8s'.'%.3s'\n", name83, name83 + 8);
  uint32_t root_sectors = (fatInfo.root_entry_count * 32) / fatInfo.bytes_per_sector;
  uint32_t eps = DISK_SECTOR_SIZE / sizeof(fat_dir_entry_t);
  uint32_t freeStart_sector = 0, freeStart_idx = 0, freeCount = 0;
  bool found = false;
  for (uint32_t s = 0; s < root_sectors && !found; s++) {
    uint8_t buf[DISK_SECTOR_SIZE];
    if (!readSector(fatInfo.root_start + s, buf)) return false;
    fat_dir_entry_t* entries = (fat_dir_entry_t*)buf;
    for (uint32_t i = 0; i < eps && !found; i++) {
      bool isFree = (entries[i].name[0] == 0x00 || (uint8_t)entries[i].name[0] == 0xE5);
      if (isFree) {
        if (freeCount == 0) { freeStart_sector = s; freeStart_idx = i; }
        freeCount++;
        if (freeCount >= (uint32_t)totalEntries) found = true;
      } else {
        if (entries[i].name[0] == 0x00) goto done;
        freeCount = 0;
      }
    }
  }
  done:
  if (!found) { Serial.println("[ERROR] Root directory is full"); return false; }
  for (int lfn = lfnCount, entryNum = 0; lfn >= 1; lfn--, entryNum++) {
    uint32_t absIdx = freeStart_sector * eps + freeStart_idx + entryNum;
    uint32_t s = absIdx / eps, i = absIdx % eps;
    uint8_t buf[DISK_SECTOR_SIZE];
    if (!readSector(fatInfo.root_start + s, buf)) return false;
    lfnFillEntry(&((fat_lfn_entry_t*)buf)[i], lfn, (lfn == lfnCount), utf16, nameLen, checksum);
    if (!writeSector(fatInfo.root_start + s, buf)) return false;
  }
  {
    uint32_t absIdx = freeStart_sector * eps + freeStart_idx + lfnCount;
    uint32_t s = absIdx / eps, i = absIdx % eps;
    uint8_t buf[DISK_SECTOR_SIZE];
    if (!readSector(fatInfo.root_start + s, buf)) return false;
    fat_dir_entry_t* e = &((fat_dir_entry_t*)buf)[i];
    memset(e, 0, sizeof(fat_dir_entry_t));
    memcpy(e->name, name83, 8); memcpy(e->ext, name83 + 8, 3);
    e->attr             = 0x20;
    e->first_cluster_lo = first_cluster;
    e->file_size        = file_size;
    if (!writeSector(fatInfo.root_start + s, buf)) return false;
    *out_sector = fatInfo.root_start + s; *out_idx = i;
  }
  return true;
}

static uint16_t fatWriteData(const uint8_t* data, uint32_t size) {
  if (size == 0) return 0;
  uint32_t remaining = size, dataOffset = 0;
  uint16_t firstCluster = 0xFFFF, prevCluster = 0xFFFF;
  while (remaining > 0) {
    uint16_t cluster = fatFindFreeCluster();
    if (cluster == 0xFFFF) { Serial.println("[ERROR] Partition is full"); return 0xFFFF; }
    fat12SetEntry(cluster, 0xFFF);
    if (firstCluster == 0xFFFF) firstCluster = cluster;
    if (prevCluster  != 0xFFFF) fat12SetEntry(prevCluster, cluster);
    uint32_t cs = clusterToSector(cluster);
    for (uint8_t s = 0; s < fatInfo.sectors_per_cluster && remaining > 0; s++) {
      uint8_t buf[DISK_SECTOR_SIZE]; memset(buf, 0, DISK_SECTOR_SIZE);
      uint32_t toWrite = min((uint32_t)DISK_SECTOR_SIZE, remaining);
      memcpy(buf, data + dataOffset, toWrite);
      if (!writeSector(cs + s, buf)) {
        Serial.printf("[ERROR] Failed to write data cluster %u sector %u\n", cluster, s);
        return 0xFFFF;
      }
      dataOffset += toWrite; remaining -= toWrite;
    }
    prevCluster = cluster;
  }
  return firstCluster;
}

// =============================================================================
// PUBLIC API — FILE READ INTO BUFFER
// =============================================================================

// Reads a file from the FAT partition into a RAM buffer.
// For files smaller than PSRAM_THRESHOLD (50KB), uses regular heap (SRAM).
// For larger files, uses PSRAM via ps_malloc() to avoid running out of SRAM.
// Returns a pointer to the allocated buffer and sets *outSize to the file size.
// The CALLER is responsible for freeing the buffer with free() when done.
// Returns nullptr if the file is not found or allocation fails.
uint8_t* fatReadFileToBuffer(const char* path, uint32_t* outSize) {
  if (!fatReadBootSector() || !fatLoadTable()) return nullptr;

  FatEntry entry;
  if (!fatResolvePath(path, &entry)) {
    if (DEBUG) Serial.printf("[INFO] File '%s' not found\n", path);
    return nullptr;
  }
  if (entry.is_dir) {
    Serial.printf("[ERROR] fatReadFileToBuffer: '%s' is a directory\n", path);
    return nullptr;
  }

  uint32_t size = entry.file_size;
  *outSize = size;

  if (size == 0) return nullptr;

  // Choose allocation strategy based on file size
  uint8_t* buf;
  if (size > PSRAM_THRESHOLD) {
    if (DEBUG) Serial.printf("[FAT] Allocating %lu bytes in PSRAM\n", size);
    buf = (uint8_t*)ps_malloc(size);
  } else {
    buf = (uint8_t*)malloc(size);
  }

  if (!buf) {
    Serial.printf("[ERROR] fatReadFileToBuffer: failed to allocate %lu bytes\n", size);
    return nullptr;
  }

  // Read cluster chain into buffer
  uint32_t remaining = size, offset = 0;
  uint16_t cluster = entry.first_cluster;

  while (cluster >= 0x002 && cluster <= 0xFEF && remaining > 0) {
    uint32_t cs = clusterToSector(cluster);
    for (uint8_t s = 0; s < fatInfo.sectors_per_cluster && remaining > 0; s++) {
      uint8_t localBuf[DISK_SECTOR_SIZE];
      if (!readSector(cs + s, localBuf)) {
        Serial.println("[ERROR] fatReadFileToBuffer: read error");
        free(buf); return nullptr;
      }
      uint32_t toCopy = min((uint32_t)DISK_SECTOR_SIZE, remaining);
      memcpy(buf + offset, localBuf, toCopy);
      offset += toCopy; remaining -= toCopy;
    }
    cluster = fat12GetEntry(cluster);
  }

  if (DEBUG) Serial.printf("[FAT] '%s' read into buffer (%lu bytes)\n", path, size);
  return buf;
}

// =============================================================================
// PUBLIC API — FILE WRITE
// =============================================================================

bool fatWriteFile(const char* path, const uint8_t* data, uint32_t size) {
  if (!fatReadBootSector() || !fatLoadTable()) return false;
  const char* filename = strrchr(path, '/'); filename = filename ? filename + 1 : path;
  FatEntry existing; bool exists = fatResolvePath(path, &existing);
  if (exists) {
    if (existing.is_dir) { Serial.printf("[ERROR] fatWriteFile: '%s' is a directory\n", path); return false; }
    if (existing.first_cluster >= 2) fatFreeChain(existing.first_cluster);
  }
  uint16_t firstCluster = (size > 0) ? fatWriteData(data, size) : 0;
  if (size > 0 && firstCluster == 0xFFFF) return false;
  if (exists) {
    if (!fatUpdateDirEntry(existing.dir_sector, existing.dir_entry_idx, firstCluster, size)) {
      Serial.println("[ERROR] fatWriteFile: failed to update directory entry"); return false;
    }
  } else {
    uint32_t out_sector, out_idx;
    if (!fatCreateEntryWithLFN(filename, firstCluster, size, &out_sector, &out_idx)) return false;
  }
  if (!fatFlushTable()) return false;
  if (fatTable) { free(fatTable); fatTable = nullptr; }
  fatInfo.valid = false;
  if (DEBUG) Serial.printf("[FAT] '%s' written (%lu bytes)\n", path, size);
  return true;
}

bool fatAppendFile(const char* path, const uint8_t* data, uint32_t size) {
  if (!fatReadBootSector() || !fatLoadTable()) return false;
  if (size == 0) return true;
  FatEntry entry;
  if (!fatResolvePath(path, &entry)) return fatWriteFile(path, data, size);
  if (entry.is_dir) { Serial.printf("[ERROR] fatAppendFile: '%s' is a directory\n", path); return false; }
  uint32_t oldSize = entry.file_size, newSize = oldSize + size;
  uint8_t* buf = (uint8_t*)(newSize > PSRAM_THRESHOLD ? ps_malloc(newSize) : malloc(newSize));
  if (!buf) { Serial.printf("[ERROR] fatAppendFile: out of memory (%lu bytes)\n", newSize); return false; }
  uint32_t remaining = oldSize, offset = 0;
  uint16_t cluster = entry.first_cluster;
  while (cluster >= 0x002 && cluster <= 0xFEF && remaining > 0) {
    uint32_t cs = clusterToSector(cluster);
    for (uint8_t s = 0; s < fatInfo.sectors_per_cluster && remaining > 0; s++) {
      uint8_t tmp[DISK_SECTOR_SIZE];
      if (!readSector(cs + s, tmp)) { free(buf); return false; }
      uint32_t toCopy = min((uint32_t)DISK_SECTOR_SIZE, remaining);
      memcpy(buf + offset, tmp, toCopy); offset += toCopy; remaining -= toCopy;
    }
    cluster = fat12GetEntry(cluster);
  }
  memcpy(buf + oldSize, data, size);
  bool result = fatWriteFile(path, buf, newSize);
  free(buf);
  return result;
}

// =============================================================================
// PUBLIC API — DIRECTORY LISTING
// =============================================================================

static void listCallback(FatEntry* e, void* c);

static void _listDir(uint16_t cluster, bool is_root, int indent, bool recursive) {
  ListCtx ctx = { indent, recursive };
  fatIterateDir(cluster, is_root, listCallback, &ctx);
}

static void listCallback(FatEntry* e, void* c) {
  ListCtx* ctx = (ListCtx*)c;
  if (strcmp(e->longName, ".") == 0 || strcmp(e->longName, "..") == 0) return;
  for (int i = 0; i < ctx->indent; i++) Serial.print("  ");
  if (e->is_dir) {
    Serial.printf("[%s]\n", e->longName);
    if (ctx->recursive) _listDir(e->first_cluster, false, ctx->indent + 1, true);
  } else {
    Serial.printf("%s (%lu bytes)\n", e->longName, e->file_size);
  }
}

void listDirectory(const char* path, bool recursive) {
  if (!fatReadBootSector() || !fatLoadTable()) return;
  Serial.printf("[DIR] '%s'%s:\n", path, recursive ? " (recursive)" : "");
  if (strcmp(path, "/") == 0) {
    _listDir(0, true, 0, recursive);
  } else {
    FatEntry dir;
    if (!fatResolvePath(path, &dir)) { Serial.printf("[ERROR] listDirectory: '%s' not found\n", path); return; }
    if (!dir.is_dir) { Serial.printf("[ERROR] listDirectory: '%s' is not a directory\n", path); return; }
    _listDir(dir.first_cluster, false, 0, recursive);
  }
}

// =============================================================================
// APPLICATION — CONFIG AND DATA MANAGEMENT
// =============================================================================

// Reads config.json. If it does not exist, creates it with default values.
// Prints the config contents to Serial.
void handleConfig() {
  uint32_t size;
  uint8_t* buf = fatReadFileToBuffer(CONFIG_FILE, &size);

  if (!buf) {
    // File does not exist — create with defaults
    Serial.println("[CONFIG] config.json not found - creating with defaults");
    JsonDocument doc;
    doc["ssid"]     = DEFAULT_SSID;
    doc["password"] = DEFAULT_PASSWORD;
    char json[128];
    serializeJson(doc, json, sizeof(json));
    if (fatWriteFile(CONFIG_FILE, (const uint8_t*)json, strlen(json))) {
      Serial.println("[CONFIG] config.json created:");
      Serial.println(json);
    } else {
      Serial.println("[ERROR] Failed to create config.json");
    }
    return;
  }

  // File exists — parse and print
  Serial.println("[CONFIG] config.json contents:");
  Serial.write(buf, size);
  Serial.println();

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, buf, size);
  if (err) {
    Serial.printf("[ERROR] Failed to parse config.json: %s\n", err.c_str());
  } else {
    Serial.printf("[CONFIG] ssid     = %s\n", doc["ssid"].as<const char*>());
    Serial.printf("[CONFIG] password = %s\n", doc["password"].as<const char*>());
  }

  free(buf);
}

// Reads data.json, extracts the count from the last line, and appends a new
// entry with count+1 and the current timestamp (millis).
// If data.json does not exist, creates it with the first entry.
void handleData() {
  uint32_t size;
  uint8_t* buf = fatReadFileToBuffer(DATA_FILE, &size);

  int lastCount = 0;

  if (buf) {
    // Find the last non-empty line
    // Add null terminator to work with string functions
    uint8_t* text = (uint8_t*)realloc(buf, size + 1);
    if (!text) { free(buf); return; }
    buf = text;
    buf[size] = '\0';

    // Walk backwards to find the last line
    char* lastLine = nullptr;
    char* p = (char*)buf + size - 1;
    while (p > (char*)buf && (*p == '\n' || *p == '\r')) p--;  // skip trailing newlines
    while (p > (char*)buf && *p != '\n') p--;                  // find start of last line
    if (*p == '\n') p++;
    lastLine = p;

    // Parse last line as JSON
    if (lastLine && strlen(lastLine) > 0) {
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, lastLine);
      if (!err && doc["count"].is<int>()) {
        lastCount = doc["count"].as<int>();
        if (DEBUG) Serial.printf("[DATA] Last count = %d\n", lastCount);
      }
    }
    free(buf);
  } else {
    Serial.println("[DATA] data.json not found - creating");
  }

  // Append new entry
  JsonDocument newDoc;
  newDoc["timestamp"] = millis();
  newDoc["count"]     = lastCount + 1;

  char newLine[128];
  serializeJson(newDoc, newLine, sizeof(newLine));
  strncat(newLine, "\n", sizeof(newLine) - strlen(newLine) - 1);

  if (fatAppendFile(DATA_FILE, (const uint8_t*)newLine, strlen(newLine))) {
    Serial.printf("[DATA] Appended: %s", newLine);
  } else {
    Serial.println("[ERROR] Failed to write to data.json");
  }
}

// =============================================================================
// SETUP AND LOOP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[BOOT] ESP32-S3 USB MSC + FAT12");

  USB.onEvent(usbEventCallback);

  msc.vendorID("ESP32-S3");
  msc.productID("FAT Storage");
  msc.productRevision("1.0");
  msc.onRead(onRead);
  msc.onWrite(onWrite);
  msc.onStartStop(onStartStop);
  msc.mediaPresent(true);
  msc.isWritable(true);
  msc.begin(DISK_SECTOR_COUNT, DISK_SECTOR_SIZE);

  USB.begin();

  // Read config on boot
  delay(1000);
  if (!usbMounted) handleConfig();

  lastReadTime = millis();
}

void loop() {
  // Detect falling edge: USB was connected, now disconnected/ejected
  if (usbMountedPrev && !usbMounted) {
    Serial.println("[LOG] USB removed");
    handleData();
  }
  usbMountedPrev = usbMounted;

  if (!usbMounted) {
    if (millis() - lastReadTime >= READ_INTERVAL_MS) {
      listDirectory("/", true);
      lastReadTime = millis();
    }
  }
}
