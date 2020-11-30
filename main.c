#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <inttypes.h>
#include "modbusPC.c"
#include <stdbool.h>
#include <string.h>
#include <time.h>

int port; // файловый дескриптор com порта
int SerialBytes; // число реально прочитанных данных с порта
bool readIloop = true;
uint8_t mBusDev[2] = {0x11, 0x12}; // id и колличество устройств на линии
uint8_t buf[128]; // буффер com порта
uint16_t data[256]; // буффер данных

//TEMP//
uint8_t mID = 0x11; // id устройства
//TEMP//

static struct termios old_options; // структура для текущих параметров консоли порта

bool openComPort (const char *com_name, speed_t speed) {
	printf("LOG: Trying to open %s device..", com_name);
	port = open(com_name, O_RDWR | O_NOCTTY); // открываем порт в режиме чтения и записи и не терминал
	if (port < 0) {
		printf("Error opening port\n");
		return false;
	}
	else {
		printf("Done!\n");
	}

	struct termios options; // структура настроек порта

	tcgetattr(port, &old_options); // записываем текущие настройки
	options = old_options;

	cfsetispeed(&options,B57600); // baudrate приёма
	cfsetospeed(&options,B57600); // baudrate передачи
	options.c_cc[VTIME] = 1;
	options.c_cc[VMIN] = 1;

	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag &= ~HUPCL; // что бы порт не закрывался, во избежание перезагрузки Ardionp
	options.c_cflag |= CS8;
	

	//options.c_oflag &= ~OPOST;
	options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	//options.c_iflag &= ~(IXON | IXOFF | IXANY);
	//options.c_cflag |= CREAD | CLOCAL;
	//options.c_cflag &= ~CRTSCTS;

	tcsetattr(port, TCSANOW, &options); // устанавливаем параметры консоли
	
	//printf("LOG: Waiting 3 seconds for Arduino Reset..");
	usleep(3*1000000); // задержка в 3 секунды, что бы Arduino успела перезагрузиться
	return true; // всё успешно
}

void writeCom (uint8_t n) { // пишем в порт нужный кусок буфера
	write(port, buf, n);
}

bool checkError (uint8_t fn, uint8_t id) { // ловим ошибки
	if (!checkDU(buf, SerialBytes)) { // неверная контрольная сумма
		printf("LOG: ERROR! Wrong CRC or device %#02x is down!\n\n", id);
		return true;
	}
	if (buf[1] == fn) { // принятый от устройства номер функции соответствует переданному(то есть не ошибка)
		return false;
	}
	if (buf[1] == (fn | 0x80) && (buf[1] != 0)) { // в протоколе mbus так возвращается ошибка
		printf("LOG: ERROR while doing %#02x function on %#02x device!\n", fn, id);
		if (buf[2] == 0x01) {
			printf("MSG: ModBus error 0x01 - Function unsupported!\n");
			return true;
		}
		else if (buf[2] == 0x02) {
			printf("MSG: ModBus error 0x02 - Illegal data address!\n");
			return true;
		}
		else if (buf[2] == 0x03) {
			printf("ModBus error 0x03 - Illegal data value!\n");
			return true;
		}
		else if (buf[2] == 0x04) {
			if (buf[3] == 0x01) {
				printf("MSG: MotorSpy Error 0x01 - The device isn't ready!\n");
			}
			else {
				printf("MSG: ModBus error 0x04 - Unknown error!\n");
				return true;
			}
 		}
	}
	else if ((buf[1] != fn) &&(buf[1] != 0)) {
		printf("LOG: ERROR while doing %#02x function on %#02x device!\n", fn, id);
		printf("LOG: The device returns another function code!\n");
		return true;
	}
	else if (buf[1] == 0) { // ментальная пустота внутри ответа
		printf("LOG: ERROR while doing %#02x function on %#02x device!\n", fn, id);
		printf("LOG: Empty response code!\n");
		return true;
	}
}
void readCom () { // читаем из порта
	memset(&buf, 0, sizeof(buf)); // куда же без очистки буффффера
	usleep(0.8*1000000); // задержка в 0.2 сек для того, что бы буффер порта успел заполниться
	SerialBytes = read(port, &buf, sizeof(buf));
	//usleep(0.8*1000000); // задержка в 0.2 сек для того, что бы буффер порта успел заполниться
	return;
}

bool readHolding (uint8_t id, uint16_t from, uint16_t count) { // читаем Holding Registers
	uint8_t n = 8; // 1-I 1-F 2-from 2-count 2-CC
	memset(&buf, 0, sizeof(buf)); // православная очистка буффера

	buf[0] = id; // modbus id
	buf[1] = 0x03; // код функции чтения holding registers
	buf[2] = from & 0x00FF; // записываем сна4ала младший байт адреса первого регистра, как это делают сейчас все.
	buf[3] = from >> 8; // затем старший байт. Да, можно сразу тут менять порядок, но для логичности понимания кода используем дальше swapByte.
	buf[4] = count & 0x00FF; // младший байт колличества байт для чтения
	buf[5] = count >> 8; // старший байт колличества
	swapByteOrder(buf, 2, 4); // меняем с LittleEndian на BigEndian перед отправкой
	signDU(buf, n); // считаем контрольную сумму и добавляем её (2 байта) в конец сообщения
	writeCom(n); // пишем в порт нужное колличество байт из свежеподготовленного массива

	readCom(); // читаем
	if (checkError(0x03, id)) { // если нашли ошибку то всё вокруг ложь!
		return false;
	}
	uint8_t recived_count = buf[2];
	swapByteOrder(buf, 3, recived_count);

	// вывод содержимого пакета на экран
	printf("Device ID: %#02x\nFunction: %#02x\nQuantity of bytes: %d\n", buf[0], buf[1], buf[2]);
	for (uint8_t i = 3; i<recived_count+3; i++) {
		printf("Value %d: %#02x\n", i-3, buf[i]);
	}
	//data[0] = ((uint16_t)buf[4] << 8 | buf[3]);
	printf("CRC byte 1: %#02x\nCRC byte 2: %#02x\n", buf[SerialBytes-2], buf[SerialBytes-1]);
	return true; // всё успешно

}

bool readInput (uint8_t id, uint16_t from, uint16_t count) {
	uint8_t n = 8; // 1-I 1-F 2-from 2-count 2-CC
	memset(&buf, 0, sizeof(buf)); // православная очистка буффера

	buf[0] = id; // modbus id
	buf[1] = 0x03; // код функции чтения holding registers
	buf[2] = from & 0x00FF; // записываем сна4ала младший байт адреса первого регистра, как это делают сейчас все.
	buf[3] = from >> 8; // затем старший байт. Да, можно сразу тут менять порядок, но для логичности понимания кода испо>
	buf[4] = count & 0x00FF; // младший байт колличества байт для чтения
	buf[5] = count >> 8; // старший байт колличества
	swapByteOrder(buf, 2, 4); // меняем с LittleEndian на BigEndian перед отправкой
	signDU(buf, n); // считаем контрольную сумму и добавляем её (2 байта) в конец сообщения
	writeCom(n); // пишем в порт нужное колличество байт из свежеподготовленного массива

	readCom(); // читаем
	if (checkError(0x03, id)) { // если нашли ошибку то всё вокруг ложь!
		return false;
	}
	uint8_t recived_count = buf[2];
	swapByteOrder(buf, 3, recived_count);

	// вывод содержимого пакета на экран
	printf("ID: %#02x \nFN: %#02x \n", buf[0], buf[1]);
	for (uint8_t i = 3; i<recived_count+3; i++) {
		printf("%d: %#02x \n", i-3, buf[i]);
	}
	/*for (uint8_t i = 2; i<(recived_count+3)/2; i++) {
		data[i-2] = ((uint16_t)buf[i*2] << 8 | buf[(i*2)-1]);
		printf("D%d: %d ", i, data[i-2]);
	}
	*/
	//data[0] = ((uint16_t)buf[4] << 8 | buf[3]);
	printf("CRC1: %#02x \nCRC2: %#02x\n\n", buf[SerialBytes-2], buf[SerialBytes-1]);
	return true; // всё успешно
}

bool writeEeprom (uint8_t id) { // передаём команду для записи временных настроек, переданных ранее, в EEPROM
	uint8_t n = 6;
	memset(&buf, 0, sizeof(buf)); // православная очистка буффера
	buf[0] = id; // modbus id
	buf[1] = 0x41; // код пользоавательской функции
	buf[2] = 0; // 1-й байт расширения функции P.S. я не тупой, и понимаю, что после очистки буфера там и так будут нули, написано для логичности
	buf[3] = 0; // 2-й байт расширения функции
	signDU(buf, n);
	writeCom(n);
	readCom();
	if (checkError(0x41, id)) { // если нашли ошибку то всё вокруг ложь!
		return false;
	}
	if (buf[3] == 2) {
		swapByteOrder(buf, 4, 2);
		uint16_t sizeE = ((uint16_t)buf[5] << 8 | buf[4]);
		printf("EEPROM HAS BEEN SUCCESSFULY BURNED!\n");
		printf("EEPROM Size is: %d bytes\n", sizeE);
	}
	else {
		printf("Error!\n");
	}
	return true;
}


bool ledON (uint8_t id) {
	uint8_t n = 6;
	memset(&buf, 0, sizeof(buf)); // православная очистка буффера

	buf[0] = id; // modbus id
        buf[1] = 0x41; // код пользовательской функции
	buf[2] = 1; // 1-й байт расширения функции
	buf[3] = 1; // 2-й байт расширения функции
	signDU(buf, n);
	writeCom(n);
	readCom();
	if (checkError(0x41, id)) { // если нашли ошибку то всё вокруг ложь!
                return false;
        }
	if (buf[3] == 10) {
		printf("MSG: LED ON Operation Success!\n");
	}
	else {
		printf("MSG: Error!\n");
	}
	return true;
}

bool ledOFF (uint8_t id) {
	uint8_t n = 6;
	memset(&buf, 0, sizeof(buf)); // православная очистка буффера

	buf[0] = id; // modbus id
	buf[1] = 0x41; // код пользовательской функции
	buf[2] = 2; // 1-й байт расширения функции
	buf[3] = 1; // 2-й байт расширения функции
	signDU(buf, n);
	writeCom(n);
	readCom();
	if (checkError(0x41, id)) { // если нашли ошибку то всё вокруг ложь!
		return false;
	}
	if (buf[3] == 10) {
		printf("MSG: LED OFF Operation Success!\n");
	}
	return true;
}

int main (int argc, const char *argv[]) { // main животворящий

	printf("DrunkBatya's MotorSpy Server Daemon v 0.1b\n\n"); // сообщение в системный лог

	if (argc <= 1) { // если пустой аргумент вызова
		printf("Empty args!\nUsage %s, [command]\nTo get help use %s -h\n", argv[0], argv[0]);
		return 0;
	}

	if (strcmp(argv[1], "readI") == 0) { // если надо считать Holding Input
		if (!openComPort("/dev/ttyACM0", B57600)) { // открываем порт
			return 0;
		}
		while(readIloop) {
			for (uint8_t i = 0; i<sizeof(mBusDev); i++) {
				readInput(mBusDev[i], 0, 10);
				usleep(0.5*1000000); // задержка в 0.5 сек перед опросом следующего устройства
			}
		}
		return 0;
	}
	
	else if (strcmp(argv[1], "readH") == 0) { // если надо считать Holding Registers
		if (!openComPort("/dev/ttyACM0", B57600)) { // открываем порт
			return 0;
		}
		for (uint8_t i = 0; i<sizeof(mBusDev); i++) {
			readHolding(mBusDev[i], 0, 10);
			usleep(0.5*1000000); // задержка в 0.5 сек перед опросом следующего устройства
		}
		return 0;
	}
	
	else if (strcmp(argv[1], "writeM") == 0) { // если надо передать временные параметры
                printf("This function will be available soon..\n");
		return 0;
	}
	
	else if (strcmp(argv[1], "writeE") == 0) { // если надо временные параметры прожечь в EEPROM
		if (!openComPort("/dev/ttyACM0", B57600)) { // открываем порт
			return 0;
		}
		writeEeprom(mID);
		return 0;
	}
	else if (strcmp(argv[1], "ledON") == 0) {
		if (!openComPort("/dev/ttyACM0", B57600)) { // открываем порт
			return 0;
		}
		ledON(mID);
		return 0;
	}
	else if (strcmp(argv[1], "ledOFF") == 0) {
		if (!openComPort("/dev/ttyACM0", B57600)) { // открываем порт
			return 0;
		}
		ledOFF(mID);
		return 0;
	}
	else if (strcmp(argv[1], "-h") == 0) { // справочка
		printf("Usage %s, [command]\n", argv[0]);
		printf("List of commands:\n");
		printf("	[readI] - To read Input Registers;\n	[readH] - To read Holding Registers;\n");
		printf("	[writeM] - To write temp settings;\n	[writeE] - To burn temp settings into EEPROM\n");
		printf("	[ledON] - To power on the debug led;\n	[ledOFF] - To shutdown debug led;\n");
		printf("	[-h] - To get this help;\n");
		return 0;
    }
	else { // если надо сделать то, что находится за пределом границ создания автора сего кода
		printf("Wrong command!\nUsage %s, [command]\nTo get help use %s -h\n", argv[0], argv[0]);
		return 0;
	}

	//tcsetattr(port, TCSANOW, &old_options); // когда закончили насиловать com порт, вертаем всё в зад

	close(port); // позволим насиловать порт кому-то другому
	return 0; // РЕТУРН! 
}
