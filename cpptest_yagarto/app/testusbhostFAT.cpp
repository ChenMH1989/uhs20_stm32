/*
 * testusbhostFAT.c
 *
 *  Created on: 2013Äê8ÔÂ21ÈÕ
 *      Author: Hz
 */

#include <bsp.h>
#include <Usb.h>
#include <masstorage.h>
#include <Storage.h>

#include <PCpartition/PCPartition.h>
#include <FAT/FAT.h>
#include <string.h>


#define true 0x1
#define false 0x0

volatile int brightness = 0; // how bright the LED is
volatile int fadeAmount = 80; // how many points to fade the LED by
volatile uint8_t current_state = 1;
volatile uint32_t LEDnext_time; // fade timeout
volatile uint8_t last_state = 0;
volatile uint8_t fatready = false;
volatile uint8_t partsready = false;
volatile uint32_t HEAPnext_time; // when to print out next heap report
volatile uint8_t runtest = false;
volatile uint8_t usbon = false;
volatile uint32_t usbon_time;
volatile uint8_t change = false;
volatile uint8_t reportlvl = false;
int cpart = 0;
PCPartition *PT;
FIL My_File_Object_x; /* File object */

static PFAT *Fats[_VOLUMES];
static part_t parts[_VOLUMES];
static storage_t sto[_VOLUMES];

/*make sure this is a power of two. */
#define mbxs 128
static uint8_t My_Buff_x[mbxs]; /* File read buffer */


BulkOnly *Bulk[MAX_USB_MS_DRIVERS];

/**
 * This must be called before using generic_storage. This works around a G++ bug.
 * Thanks to Lei Shi for the heads up.
 */
void InitStorage(void) {

    for (int i = 0; i < _VOLUMES; i++) {
		Fats[i] = NULL;
		sto[i].private_data = new pvt_t;
		((pvt_t *)sto[i].private_data)->B = 255; // impossible
    }

	for(int i=0; i< MAX_USB_MS_DRIVERS; i++) {
		Bulk[i]= new BulkOnly(&Usb);
	}
}

bool isfat(uint8_t t) {
	return (t == 0x01 || t == 0x04 || t == 0x06 || t == 0x0b || t == 0x0c || t == 0x0e || t == 0x1);
}

void die(FRESULT rc) {
        printf(PSTR("Failed with rc=%u.\r\n"), rc);
        //for (;;);
}

void check_fatstatus(void) {
    current_state = Usb.getUsbTaskState();
	if (current_state != last_state) {
		if (UsbDEBUGlvl > 0x50)
			printf(PSTR("USB state = 0x%x\r\n"), current_state);
		if (current_state == USB_STATE_RUNNING) {
			fadeAmount = 30;
		}
		if (current_state == USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE) {
			fadeAmount = 80;
			partsready = false;
			for (int i = 0; i < cpart; i++) {
				if (Fats[i] != NULL)
					delete Fats[i];
				Fats[i] = NULL;
			}
			fatready = false;
			cpart = 0;
		}
		last_state = current_state;
	}

	if (partsready && !fatready) {
		if (cpart > 0) fatready = true;
	}

    // This is horrible, and needs to be moved elsewhere!
	for (int B = 0; B < MAX_USB_MS_DRIVERS; B++) {
		if (!partsready && Bulk[B]->GetAddress() != NULL) {
				// Build a list.
			int ML = Bulk[B]->GetbMaxLUN();
			//printf("MAXLUN = %i\r\n", ML);
			ML++;
			for (int i = 0; i < ML; i++) {
				if (Bulk[B]->LUNIsGood(i)) {
					partsready = true;
					((pvt_t *)sto[i].private_data)->lun = i;
					((pvt_t *)sto[i].private_data)->B = B;
					sto[i].Read = *PRead;
					sto[i].Write = *PWrite;
					sto[i].Reads = *PReads;
					sto[i].Writes = *PWrites;
					sto[i].Status = *PStatus;
					sto[i].TotalSectors = Bulk[B]->GetCapacity(i);
					sto[i].SectorSize = Bulk[B]->GetSectorSize(i);
					printf(PSTR("LUN:\t\t%u\r\n"), i);
					printf(PSTR("Total Sectors:\t0x%x\t%d\r\n"), sto[i].TotalSectors, sto[i].TotalSectors);
					printf(PSTR("Sector Size:\t0x%x\t\t%d\r\n"), sto[i].SectorSize, sto[i].SectorSize);
					// get the partition data...
					PT = new PCPartition;

					if (!PT->Init(&sto[i])) {
							part_t *apart;
							for (int j = 0; j < 4; j++) {
								apart = PT->GetPart(j);
								if (apart != NULL && apart->type != 0x00) {
									memcpy(&(parts[cpart]), apart, sizeof (part_t));
									printf(PSTR("Partition %u type %#02x\r\n"), j, parts[cpart].type);
									// for now
									if (isfat(parts[cpart].type)) {
										Fats[cpart] = new PFAT;
										int r = Fats[cpart]->Init(&sto[i], cpart, parts[cpart].firstSector);
										if (r) {
											delete Fats[cpart];
											Fats[cpart] = NULL;
										} else cpart++;
									}
								}
							}
					} else {
						// try superblock
						Fats[cpart] = new PFAT;
						int r = Fats[cpart]->Init(&sto[i], cpart, 0);
						if (r) {
							printf(PSTR("Superblock error %x\r\n"), r);
							delete Fats[cpart];
							Fats[cpart] = NULL;
						} else cpart++;

					}
					delete PT;
				} else {
					sto[i].Read = NULL;
					sto[i].Write = NULL;
					sto[i].Writes = NULL;
					sto[i].Reads = NULL;
					sto[i].TotalSectors = 0UL;
					sto[i].SectorSize = 0;
				}
			}

		}
	}

	 if (fatready) {
		if (Fats[0] != NULL) {
			struct Pvt * p;
			p = ((struct Pvt *)(Fats[0]->storage->private_data));
			if (!Bulk[p->B]->LUNIsGood(p->lun)) {
				// media change
				fadeAmount = 80;
				partsready = false;
				for (int i = 0; i < cpart; i++) {
					if (Fats[i] != NULL)
						delete Fats[i];
					Fats[cpart] = NULL;
				}
				fatready = false;
				cpart = 0;
			}

		}
	}
}
void demo_fileoperation(void) {
	if (fatready) {
		FRESULT rc; /* Result code */
		UINT bw, br, i;

		fadeAmount = 5;
#if 0
		printf(PSTR("\r\nOpen an existing file (message.txt).\r\n"));
		rc = f_open(&My_File_Object_x, "0:/MESSAGE.TXT", FA_READ);
		if (rc) printf(PSTR("Error %i, message.txt not found.\r\n"));
		else {
			printf(PSTR("\r\nType the file content.\r\n"));
			for (;;) {
				rc = f_read(&My_File_Object_x, My_Buff_x, mbxs, &br); /* Read a chunk of file */
				if (rc || !br) break; /* Error or end of file */
				for (i = 0; i < br; i++) {
					/* Type the data */
					if (My_Buff_x[i] == '\n')
						printf("\r");
					if (My_Buff_x[i] != '\r')
						printf("%c", My_Buff_x[i]);
					//Serial.flush();
				}
			}
			if (rc) {
				f_close(&My_File_Object_x);
				goto out;
			}

			printf(PSTR("\r\nClose the file.\r\n"));
			rc = f_close(&My_File_Object_x);
			if (rc) goto out;
		}
#endif
		printf(PSTR("\r\nCreate a new file (hello.txt).\r\n"));
		rc = f_open(&My_File_Object_x, "0:/Hello.TxT", FA_WRITE | FA_CREATE_ALWAYS);
		if (rc) {
			die(rc);
			goto out;
		}
		printf(PSTR("\r\nWrite a text data. (Hello world!)\r\n"));
		rc = f_write(&My_File_Object_x, "Hello world!\r\n16:12 2013/8/23\r\r\n\nHozen", 30, &bw);
		if (rc) {
			goto out;
		}
		printf(PSTR("%u bytes written.\r\n"), bw);

		printf(PSTR("\r\nClose the file.\r\n"));
		rc = f_close(&My_File_Object_x);
		if (rc) {
			die(rc);
			goto out;
		}

out:
		if (rc) die(rc);
		printf(PSTR("\r\nTest completed.\r\n"));
	}

}

void demo_directorybrowse(void) {

    if (fatready) {
		FRESULT rc; /* Result code */
		UINT bw, br, i;

outdir:
#if _USE_LFN
		char lfn[_MAX_LFN + 1];
		FILINFO My_File_Info_Object_x; /* File information object */
		My_File_Info_Object_x.lfname = lfn;
#endif
		DIR My_Dir_Object_x; /* Directory object */
		printf(PSTR("\r\nOpen root directory.\r\n"));
		rc = f_opendir(&My_Dir_Object_x, "0:/");
		if (rc) {
			die(rc);
			goto out;
		}

		printf(PSTR("\r\nDirectory listing...\r\n"));
		for (;;) {
#if _USE_LFN
			My_File_Info_Object_x.lfsize = _MAX_LFN;
#endif

			rc = f_readdir(&My_Dir_Object_x, &My_File_Info_Object_x); /* Read a directory item */
			if (rc || !My_File_Info_Object_x.fname[0]) break; /* Error or end of dir */

			if (My_File_Info_Object_x.fattrib & AM_DIR) {
				printf("d");
			} else {
				printf("-");
			}
			printf("r");

			if (My_File_Info_Object_x.fattrib & AM_RDO) {
				printf("-");
			} else {
				printf("w");
			}
			if (My_File_Info_Object_x.fattrib & AM_HID) {
				printf("h");
			} else {
				printf("-");
			}

			if (My_File_Info_Object_x.fattrib & AM_SYS) {
				printf("s");
			} else {
				printf("-");
			}

			if (My_File_Info_Object_x.fattrib & AM_ARC) {
				printf("a");
			} else {
				printf("-");
			}

#if _USE_LFN
			if (*My_File_Info_Object_x.lfname)
				printf(PSTR(" %d  %s (%s)\r\n"), My_File_Info_Object_x.fsize, My_File_Info_Object_x.fname, My_File_Info_Object_x.lfname);
			else
#endif
				printf(PSTR(" %d  %s\r\n"), My_File_Info_Object_x.fsize, &(My_File_Info_Object_x.fname[0]));
		}
out:
		if (rc) die(rc);
		printf(PSTR("\r\nTest completed.\r\n"));

	}

}

void demo_speedtest(void) {
	if(fatready) {
		FRESULT rc; /* Result code */
		UINT bw, br, i;

		ULONG ii, wt, rt, start, end;
		runtest = false;
		f_unlink("0:/10MB.bin");
		printf(PSTR("\r\nCreate a new 10MB test file (10MB.bin).\r\n"));
		rc = f_open(&My_File_Object_x, "0:/10MB.bin", FA_WRITE | FA_CREATE_ALWAYS);
		if (rc) goto failed;
		for (bw = 0; bw < mbxs; bw++) My_Buff_x[bw] = bw & 0xff;
		start = millis();
		for (ii = 10485760LU / mbxs; ii > 0LU; ii--) {
				rc = f_write(&My_File_Object_x, My_Buff_x, mbxs, &bw);
				if (rc || !bw) goto failed;
		}
		rc = f_close(&My_File_Object_x);
		if (rc) goto failed;
		end = millis();
		wt = end - start;
		printf(PSTR("Time to write 10485760 bytes: %d ms (%d sec) \r\n"), wt, (500 + wt) / 1000UL);
		rc = f_open(&My_File_Object_x, "0:/10MB.bin", FA_READ);
		start = millis();
		if (rc) goto failed;
		for (;;) {
				rc = f_read(&My_File_Object_x, My_Buff_x, mbxs, &bw); /* Read a chunk of file */
				if (rc || !bw) break; /* Error or end of file */
		}
		end = millis();
		if (rc) goto failed;
		rc = f_close(&My_File_Object_x);
		if (rc) goto failed;
		rt = end - start;
		printf(PSTR("Time to read 10485760 bytes: %d ms (%d sec)\r\nDelete test file\r\n"), rt, (500 + rt) / 1000UL);
failed:
		if (rc) die(rc);
		printf(PSTR("10MB timing test finished.\r\n"));
	}
}

bool PStatus(storage_t *sto) {
        return (Bulk[((pvt_t *)sto->private_data)->B]->WriteProtected(((pvt_t *)sto->private_data)->lun));
}

int PRead(uint32_t LBA, uint8_t *buf, storage_t *sto) {
        uint8_t x = 0;
        int tries = FAT_MAX_ERROR_RETRIES;
        while (tries) {
                tries--;
                x = (Bulk[((pvt_t *)sto->private_data)->B]->Read(((pvt_t *)sto->private_data)->lun, LBA, (sto->SectorSize), 1, buf));
                if (!x) break;
        }
        int y = x;
        return y;
}

int PWrite(uint32_t LBA, uint8_t *buf, storage_t *sto) {
        int x = 0;
        int tries = FAT_MAX_ERROR_RETRIES;
        while (tries) {
                tries--;
                x = (Bulk[((pvt_t *)sto->private_data)->B]->Write(((pvt_t *)sto->private_data)->lun, LBA, sto->SectorSize, 1, buf));
                if (x == MASS_ERR_WRITE_PROTECTED) break;
                if (!x) break;
        }
        int y = x;
        return y;
}

int PReads(uint32_t LBA, uint8_t *buf, storage_t *sto, uint8_t count) {
        uint8_t x = 0;
        int tries = FAT_MAX_ERROR_RETRIES;
        while (tries) {
                tries--;
                x = (Bulk[((pvt_t *)sto->private_data)->B]->Read(((pvt_t *)sto->private_data)->lun, LBA, (sto->SectorSize), count, buf));
                if (!x) break;
                delay(200);
        }
        int y = x;
        return y;
}

int PWrites(uint32_t LBA, uint8_t *buf, storage_t *sto, uint8_t count) {
        int x = 0;
        int tries = FAT_MAX_ERROR_RETRIES;
        while (tries) {
                tries--;
                x = (Bulk[((pvt_t *)sto->private_data)->B]->Write(((pvt_t *)sto->private_data)->lun, LBA, sto->SectorSize, count, buf));
                if (x == MASS_ERR_WRITE_PROTECTED) break;
                if (!x) break;
                delay(200);
        }
        int y = x;
        return y;
}
