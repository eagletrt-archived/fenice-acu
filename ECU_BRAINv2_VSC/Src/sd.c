#include "sd.h"

//--- RESULT VARIABLES FOR SD ---//
FRESULT res_open;
FRESULT res_mount;

//--- FILE OBJECT FOR SD ---//
FIL loggingFile;
FIL pot_values_f;
FIL log_names_f;

TCHAR message[256];
char name_txt[] = "name.txt";
char filename_log[50];
char filename_pot[] = "potV.txt";
char txt[2000];

int max_files = 100;
UINT byteswritten;
int mount_ok = 0;
int msg_counter = 0;
int msg_index = 0;

int bytes_read;

char *pointer;
char log_names[1100];

uint8_t pot_values_loaded = 0;

int successfull_opening = 0;

//----------------------------------------//
//------------- FUNCTIONS ----------------//
//----------------------------------------//

void init_sd(){
    res_mount = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1); //try to mount sd card

	if (res_mount == FR_OK) {

		res_open = f_open(&log_names_f, (TCHAR const*)&name_txt, FA_OPEN_ALWAYS | FA_READ ); //open "name.txt" file, if doesn't exit create it
		if(res_open == 0)f_read(&log_names_f, log_names, 1099, (void*)&bytes_read); //read into file "name.txt" and put the result into "log_names" variable

		HAL_UART_Transmit(&huart4,(uint8_t*)"mounted, opened\r\n",strlen("mounted, opened\r\n"),10);

		// sprintf(txt, "FILENAME RES VAL: %d",(int)res_open);
		// HAL_UART_Transmit(&huart4,(uint8_t*)txt,strlen(txt),100);
		// sprintf(txt, "%s\r\n", log_names);
		// HAL_UART_Transmit(&huart4,(uint8_t*)txt,strlen(txt),100);

		char name[256];

		f_close(&log_names_f); // close the file
		f_open(&log_names_f, (TCHAR const*)&name_txt, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_WRITE );

		for(int i = 0; i < max_files; i++){

			sprintf(name, "log_%d ", i);

			pointer = strstr(log_names, name);

			if(i == max_files){

				sprintf(filename_log,"default.txt");

				f_open(&loggingFile, (TCHAR const*)&filename_log, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
				f_close(&loggingFile);
				//f_open(&loggingFile, (TCHAR const*)&filename, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );

				HAL_UART_Transmit(&huart4,(uint8_t*)"created -> default.txt\r\n",strlen("created -> default.txt\r\n"),10);

				successfull_opening = 1;

				break;
			}

			if(i == 0 && pointer == NULL){

				sprintf(filename_log, "log_0 \r\n");

				f_write(&log_names_f, filename_log, strlen(filename_log), &byteswritten);
				f_close(&log_names_f);

				sprintf(filename_log, "Log_0.txt");

				f_open(&loggingFile, (TCHAR const*)&filename_log, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
				f_close(&loggingFile);
				//f_open(&loggingFile, (TCHAR const*)&filename, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );

				HAL_UART_Transmit(&huart4,(uint8_t*)"\r\ncreated -> Log_0.txt\r\n",strlen("\r\ncreated -> Log_0\r\n"),10);

				successfull_opening = 1;

				break;
			}
			if(pointer == NULL){

				sprintf(filename_log, "log_%d \r\n", i);

				f_write(&log_names_f, filename_log, strlen(filename_log), (void*)&byteswritten);
				f_close(&log_names_f);

				sprintf(filename_log, "Log_%d.txt", i);

				f_open(&loggingFile, (TCHAR const*)&filename_log, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
				f_close(&loggingFile);
				//f_open(&loggingFile, (TCHAR const*)&filename, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );

				HAL_UART_Transmit(&huart4,(uint8_t*)"\r\ncreated -> ",strlen("\r\ncreated -> "),10);
				HAL_UART_Transmit(&huart4,(uint8_t*) filename_log, strlen(filename_log), 10);
				HAL_UART_Transmit(&huart4,(uint8_t*) "\r\n", strlen("\r\n"),10);

				successfull_opening = 1;

				break;
			}
		}

		mount_ok = 1;
		HAL_UART_Transmit(&huart4,(uint8_t*)"files closed\r\n",strlen("files closed\r\n"),10);

		res_open = f_open(&pot_values_f, (TCHAR const*)&filename_pot, FA_OPEN_EXISTING | FA_READ ); //open "pot_val.txt" file, if doesn't exit create it
		char pot_values[100];
		sprintf(txt, "POT FILE VAL: %d\r\n",(int)res_open);
		HAL_UART_Transmit(&huart4,(uint8_t*)txt,strlen(txt),100);
		if(res_open == FR_OK){
			
			HAL_UART_Transmit(&huart4,(uint8_t*)"\r\nPOT FILE OPEND\r\n",strlen("\r\nPOT FILE OPEND\r\n"),10);
			f_read(&pot_values_f, pot_values, 100, (void*)&bytes_read); //read into file "pot_values.txt" and put the result into "pot_value" variable
			char val_pot[6];
			val_pot[5] = '\0';
			int pot_val_count = 0;
			for(int i=0; i<5 ; i++){
				if(pot_values[pot_val_count] == ' '){ // check if the number isn't finished
					val_pot[i] = '\0';
					i = 5; // set exit condition
					pot_val_count++;

				}else{
					val_pot[i] = pot_values[pot_val_count];
					pot_val_count++;
				}
			}
			pot1_max = atoi(val_pot);
			for(int i=0 ; i<5 ; i++){
				if(pot_values[pot_val_count] == ' '){ // check if the number isn't finished
					val_pot[i] = '\0';
					i = 5; // set exit condition
					pot_val_count++;
				}else{
					val_pot[i] = pot_values[pot_val_count];
					pot_val_count++;
				}
			}
			pot1_min = atoi(val_pot);
			for(int i=0 ; i<5 ; i++){
				if(pot_values[pot_val_count] == ' '){ // check if the number isn't finished
					val_pot[i] = '\0';
					i = 5; // set exit condition
					pot_val_count++;
				}else{
					val_pot[i] = pot_values[pot_val_count];
					pot_val_count++;
				}
			}
			pot2_max = atoi(val_pot);
			for(int i=0 ; i<5 ; i++){
				if(pot_values[pot_val_count] == ' '){ // check if the number isn't finished
					val_pot[i] = '\0';
					i = 5; // set exit condition
					pot_val_count++;
				}else{
					val_pot[i] = pot_values[pot_val_count];
					pot_val_count++;
				}
			}
			pot2_min = atoi(val_pot);
			sprintf(txt, "max1 %ld, min1 %ld, max2 %ld, min2 %ld\r\n", pot1_max, pot1_min, pot2_max, pot2_min);
			HAL_UART_Transmit(&huart4,(uint8_t*)txt, strlen(txt),10);
			pot_values_loaded = 1;

		}else if(res_open == FR_NO_FILE){
			res_open = f_open(&pot_values_f, (TCHAR const*)&filename_pot, FA_CREATE_NEW | FA_WRITE );
			sprintf(txt, "POT FILE VAL2: %d\r\n",(int)res_open);
			HAL_UART_Transmit(&huart4,(uint8_t*)txt,strlen(txt),100);
			if(res_open == FR_OK){

				char local_txt[] = "4096 0000 4096 0000 \r\n";
				f_write(&pot_values_f,(TCHAR const*)&local_txt,strlen(local_txt), &byteswritten);
				f_close(&pot_values_f);
			}
		}
		f_close(&pot_values_f);
		
	}else {
		mount_ok = 0;
	}
}
bool clean_sd(){
	bool ret = false;
	if (res_mount == FR_OK) {
			res_open = f_open(&log_names_f, (TCHAR const*)&name_txt, FA_OPEN_ALWAYS | FA_READ ); //open "name.txt" file, if doesn't exit create it
			if(res_open == 0){
				f_read(&log_names_f, log_names, 1099, (void*)&bytes_read); //read into file "name.txt" and put the result into "log_names" variable
				char name[256];
				char nameFile[256];
				for(int i = 0; i < max_files; i++){

					sprintf(name, "log_%d ", i);

					pointer = strstr(log_names, name);
					if(pointer != NULL){
						sprintf(txt,"\r\nRemoving %s ... ", name);
						HAL_UART_Transmit(&huart4,(uint8_t*)txt,strlen(txt),100);
						sprintf(nameFile, "Log_%d.txt", i);
						if(f_unlink(nameFile) == FR_OK){
							sprintf(txt,"DONE");
							HAL_UART_Transmit(&huart4,(uint8_t*)txt,strlen(txt),100);
						}else{
							sprintf(txt,"FAIL");
							HAL_UART_Transmit(&huart4,(uint8_t*)txt,strlen(txt),100);
						} 
					}else{
						sprintf(txt,"\r\nNo more file to remove\r\n");
						HAL_UART_Transmit(&huart4,(uint8_t*)txt,strlen(txt),100);
						i = max_files;
					}
					
				}
				f_close(&log_names_f);
				sprintf(txt,"\r\nRemoving log name file ... ");
				HAL_UART_Transmit(&huart4,(uint8_t*)txt,strlen(txt),100);
				if(f_unlink(name_txt) == FR_OK){
					sprintf(txt,"DONE");
					HAL_UART_Transmit(&huart4,(uint8_t*)txt,strlen(txt),100);
				}else{
					sprintf(txt,"FAIL");
					HAL_UART_Transmit(&huart4,(uint8_t*)txt,strlen(txt),100);
				} 
				ret = true;
			}
	}
	return ret;
}

void print_files_name(){
	if (res_mount == FR_OK) {
		res_open = f_open(&log_names_f, (TCHAR const*)&name_txt, FA_OPEN_ALWAYS | FA_READ ); //open "name.txt" file, if doesn't exit create it
		if(res_open == 0){
			f_read(&log_names_f, log_names, 1099, (void*)&bytes_read); //read into file "name.txt" and put the result into "log_names" variable
			char name[256];
			for(int i = 0; i < max_files; i++){

				sprintf(name, "log_%d ", i);

				pointer = strstr(log_names, name);
				if(pointer != NULL){
					sprintf(txt,"%s\r\n", name);
					HAL_UART_Transmit(&huart4,(uint8_t*)txt,strlen(txt),100);
				}else{
					i = max_files;
				}
				
			}
			f_close(&log_names_f);
		}
	}
}