#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <windows.h>
#include "mavlink/all/mavlink.h"

#define VALID_BAUDRATES_SIZE 15

// Допустимые баудрейты прописаны в winbase.h через #define (например CBR_115200)
const int valid_baudrates[VALID_BAUDRATES_SIZE] = {110,   300,   600,    1200,   2400,
                                                   4800,  9600,  14400,  19200,  38400,
                                                   56000, 57600, 115200, 128000, 256000};

int main(int argc, char *argv[]) {
  HANDLE hSerial;

  /**
   * НАСТРОЙКА COM ПОРТА
   */
  // Разбираем аргументы main() 
  if (argc != 3) {
    printf("Invalid parameters\n");
    printf("ardu-guided.exe [num of COM port] [baudrate]\n");
    return 1;
  }
  int comport_num = atoi(argv[1]);
  int baudrate = atoi(argv[2]);
  if (comport_num <= 0) {
    printf("Invalid num of COM port: %s\n", argv[1]);
    printf("ardu-guided.exe [num of COM port] [baudrate]\n");
    return 1;
  }
  boolean is_baudrate_valid = FALSE;
  for (size_t i = 0; i < VALID_BAUDRATES_SIZE; i++) {
    if (baudrate == valid_baudrates[i]) {
      is_baudrate_valid = TRUE;
      break;
    }
  }
  if (!is_baudrate_valid) {
    printf("Invalid baudrate: %s\n", argv[2]);
    printf("ardu-guided.exe [num of COM port] [baudrate]\n");
    return 1;
  }
  const size_t COMPORT_BUFF_SIZE = 15;
  char comport_buff[COMPORT_BUFF_SIZE];
  sprintf_s(comport_buff, COMPORT_BUFF_SIZE, "\\\\.\\COM%d", comport_num);
  printf("C COM: %s\n", comport_buff);
  printf("C baudrate: %s\n", argv[2]);

  // Открываем COM порт
  hSerial = CreateFile(comport_buff,GENERIC_READ | GENERIC_WRITE,
                       0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
  if (hSerial == INVALID_HANDLE_VALUE) {
    if (GetLastError() == ERROR_FILE_NOT_FOUND) {
      printf("%s does not exist\n", comport_buff);
    } else {
      printf ("Error %s opening\n", comport_buff);
    }
    return 1;
  }

  // Настраиваем COM порт
  DCB dcbSerialParams = {0};
  dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
  if (!GetCommState(hSerial, &dcbSerialParams)) {
    printf("Error getting COM port state\n");
    CloseHandle(hSerial);
    return 1;
  }
  dcbSerialParams.BaudRate = baudrate;
  dcbSerialParams.ByteSize = 8;
  dcbSerialParams.StopBits = ONESTOPBIT;
  dcbSerialParams.Parity = NOPARITY;
  if (!SetCommState(hSerial, &dcbSerialParams)) {
    printf("Error setting %s state\n", comport_buff);
    CloseHandle(hSerial);
    return 1;
  }

  // Настраиваем таймауты
  COMMTIMEOUTS timeouts={0};
  timeouts.ReadIntervalTimeout = MAXDWORD;
  timeouts.ReadTotalTimeoutConstant = 0;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  timeouts.WriteTotalTimeoutConstant = 0;
  timeouts.WriteTotalTimeoutMultiplier = 10;
  if (!SetCommTimeouts(hSerial, &timeouts)) {
    printf("Error setting %s timeouts\n", comport_buff);
    CloseHandle(hSerial);
    return 1;
  }

  /**
   * MAVLink
   */
  uint8_t drone_id = 0;
  uint8_t this_id = 254; // 255 - стандарт для наземных станций, мы будем использовать другой на всякий случай
  mavlink_heartbeat_t drone_hbeat;
  mavlink_global_position_t drone_pose;
  mavlink_statustext_t drone_text;
  int32_t target_lat = 0; // degE7
  int32_t target_lon = 0;
  float target_alt = 0; // m

  // Прием/Отправка
  uint8_t rx_byte;
  DWORD rx_real_count = 0;
  mavlink_message_t rx_msg;
  mavlink_status_t rx_status;

  mavlink_message_t tx_msg;
  uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t tx_len = 0;

  // Отслеживание для отправки
  bool drone_one_arm_check = false;
  bool drone_one_guided_check = false;
  ULONGLONG send_guided_period = 1000; // мс
  ULONGLONG send_pos_period = 1000;
  ULONGLONG send_guided_time_last = 0;
  ULONGLONG send_pos_time_last = 0;
  for (;;) {
    // ЧТЕНИЕ COM
    while (ReadFile(hSerial, &rx_byte, 1, &rx_real_count, NULL) && rx_real_count == 1) {
      // Функция mavlink_parse_char пытается собрать пакет по одному байту, используя свой внутренний буфер
      if (mavlink_parse_char(0, rx_byte, &rx_msg, &rx_status)) {
        switch (rx_msg.msgid) {
          // Это сообщение отсылается каждую секунду. По нему очень удобно находить дрон
          case MAVLINK_MSG_ID_HEARTBEAT:
            // Если компонент, отправивший HEARTBEAT является автопилотом
            if (rx_msg.compid == MAV_COMP_ID_AUTOPILOT1) {
              mavlink_heartbeat_t uncnown_hbeat;
              mavlink_msg_heartbeat_decode (&rx_msg, &uncnown_hbeat);
              // Если автопилот Ardupilot и тип дрона - мультиротор
              if (uncnown_hbeat.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA &&
                  (uncnown_hbeat.type == MAV_TYPE_QUADROTOR ||
                   uncnown_hbeat.type == MAV_TYPE_HEXAROTOR ||
                   uncnown_hbeat.type == MAV_TYPE_OCTOROTOR)) {
                if (!drone_id) {
                  drone_id = rx_msg.sysid; // Запоминаем id дрона
                  // Запрос сообщений
                  // По сути, это уже запись COM, но логично сделать здесь
                  // TODO: Т.к. при плохой связи сообщение может потеряться, логично чделать проверку,
                  //       Если передача GLOBAL_POSITION_INT не началась, то запросить сообщения повторно
                  mavlink_msg_command_long_pack(this_id, MAV_COMP_ID_MISSIONPLANNER, &tx_msg,
                                                drone_id, MAV_COMP_ID_AUTOPILOT1, MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                                MAVLINK_MSG_ID_GLOBAL_POSITION_INT,  // Что запрашиваем
                                                1000000,  // Интервал в микросекундах
                                                0, 0, 0, 0, 0);
                  tx_len = mavlink_msg_to_send_buffer(tx_buf, &tx_msg);
                  WriteFile(hSerial, tx_buf, tx_len, NULL, NULL);
                  printf("HEARTBEAT: New drone [id %d], saved\n", rx_msg.sysid);
                } else if (rx_msg.sysid == drone_id) {
                  drone_hbeat = uncnown_hbeat;
                  // Проверка для перевода в режим Guided только один раз
                  if (!drone_one_guided_check && drone_hbeat.custom_mode == COPTER_MODE_GUIDED) {
                    drone_one_guided_check = true;
                  }
                  printf("HEARTBEAT [id %d]: Mode %d, arm: %s\n", rx_msg.sysid, drone_hbeat.custom_mode,
                         (drone_hbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? "arm" : "disarm");
                } else {
                  printf("HEARTBEAT: Unknown drone [id %d]\n", rx_msg.sysid);
                }
              }
            } else {
              printf("HEARTBEAT: Not Autopilot [id %d]\n", rx_msg.compid);
            }
            break;
          case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            if (rx_msg.sysid == drone_id) {
              mavlink_msg_global_position_decode(&rx_msg, &drone_pose);
              // Внимательнее с высотой, тут есть от точки запуска и от WGS84
              printf("POSITION lat:%d, lon:%d, alt:%f\n", drone_pose.lat, drone_pose.lon, drone_pose.alt_ellipsoid);
            }
            break;
          case MAVLINK_MSG_ID_STATUSTEXT:
            if (rx_msg.sysid == drone_id) {
              mavlink_msg_statustext_decode(&rx_msg, &drone_text);
              // FIXME: Криво, потому что строка может не оканчиваться нулем, а может быть поделена на куски. Нужна проверка
              //        А пока выводим только целые сообщения
              if (drone_text.id == 0) {
                printf("TEXT: %s\n", drone_text.text);
              }
            }
            break;
          default:
            break;
        }
        // printf("RECIEVE\n");
      }
    }
    // ЗАПИСЬ COM
    if (drone_id) {
      // 1. Армим дрон
      if (!drone_one_arm_check) {
        mavlink_msg_command_long_pack(this_id, MAV_COMP_ID_MISSIONPLANNER, &tx_msg,
                                      drone_id, MAV_COMP_ID_AUTOPILOT1,
                                      MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                      1, 0, 0, 0, 0, 0, 0); // 1 - ARM
        tx_len = mavlink_msg_to_send_buffer(tx_buf, &tx_msg);
        WriteFile(hSerial, tx_buf, tx_len, NULL, NULL);
        drone_one_arm_check = true;
        printf("  SEND: Arm\n");
      }

      // 2. Переводим в режим Guided
      // if (drone_hbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED && // Раскомментировать при реальных тестах
      if (drone_one_arm_check &&
          !drone_one_guided_check &&
          drone_hbeat.custom_mode != COPTER_MODE_GUIDED &&
          (GetTickCount64() - send_guided_time_last) > send_guided_period) {
        mavlink_msg_command_long_pack(this_id, MAV_COMP_ID_MISSIONPLANNER, &tx_msg,
                                      drone_id, MAV_COMP_ID_AUTOPILOT1,
                                      MAV_CMD_DO_SET_MODE , 0,
                                      MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, COPTER_MODE_GUIDED,
                                      0, 0, 0, 0, 0);
        tx_len = mavlink_msg_to_send_buffer(tx_buf, &tx_msg);
        WriteFile(hSerial, tx_buf, tx_len, NULL, NULL);
        send_guided_time_last = GetTickCount64();
        printf("  SEND: Set mode Guided\n");
      }

      // TODO: 3. Отправляем команду на вертикальный взлет. Отслеживаем что взлетел, и только после этого отсылаем координаты

      // 4. Отправляем координаты
      if (drone_hbeat.custom_mode == COPTER_MODE_GUIDED && (GetTickCount64() - send_pos_time_last) > send_pos_period) {
        mavlink_msg_set_position_target_global_int_pack(
            this_id, MAV_COMP_ID_MISSIONPLANNER, &tx_msg,
            0,  // time_boot_ms скорее всего можно оставить 0
            drone_id, MAV_COMP_ID_AUTOPILOT1,
            MAV_FRAME_GLOBAL,  // Возможно другой фрейм
            // Битовая маска для игнорирования всех команд, кроме позиции
            // 0b110111111000
            POSITION_TARGET_TYPEMASK_VX_IGNORE |
            POSITION_TARGET_TYPEMASK_VY_IGNORE |
            POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            POSITION_TARGET_TYPEMASK_AX_IGNORE |
            POSITION_TARGET_TYPEMASK_AY_IGNORE |
            POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE,
            target_lat, target_lon, target_alt, 0, 0, 0, 0, 0, 0, 0, 0);
        tx_len = mavlink_msg_to_send_buffer(tx_buf, &tx_msg);
        WriteFile(hSerial, tx_buf, tx_len, NULL, NULL);
        send_pos_time_last = GetTickCount64();
        printf ("  SEND: Pos global lat:%d, lon:%d, alt:%f\n", target_lat, target_lon, target_alt);
      }
    }
  }

  CloseHandle(hSerial);
  return 0;
}
