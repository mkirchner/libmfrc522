#include "mfrc522.h"
#include <stdio.h>
#include <unistd.h>

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    struct mfrc522_uid uid;
    mfrc522_init();
    mfrc522_pcd_init();

    while (1) {
        // Look for a card
        if (!mfrc522_picc_is_new_card_present())
            continue;

        if (!mfrc522_picc_read_card_serial(&uid))
            continue;

        // Print UID
        for (uint8_t i = 0; i < uid.size; ++i) {
	    printf("%0x ", uid.uidByte[i]);
        }
        printf("\n");
        sleep(1);
    }
    mfrc522_deinit();
    return 0;
}
