#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <winsock2.h>
#include <windows.h>
#include <pthread.h>
#include "mavlink/all/mavlink.h"

#define SERVER_IP "127.0.0.1"
#define SERVER_PORT 5762
#define VALID_BAUDRATES_SIZE 15
#define RX_BUFF_SIZE (MAVLINK_MAX_PACKET_LEN * 4)
#define TAKEOFF_ALT 3
#define TAKEOFF_ALT_DELTA 0.5

#define PERIOD_TX_DO_NOTHING_MS   500 
#define PERIOD_TX_START_TASK_MS   5000
#define PERIOD_TX_TO_GUIDED_MS    5000
#define PERIOD_TX_ARM_MS          5000
#define PERIOD_TX_TAKEOFF_MS      10000
#define PERIOD_TX_FLY_TO_COORD_MS 2000

typedef enum {
  TASK_DO_NOTHING,
  TASK_START_TASK,
  TASK_TO_GUIDED,
  TASK_ARM,
  TASK_TAKEOFF,
  TASK_FLY_TO_COORD
} task_t;

typedef enum {
  CONNECT_NO_SELECT,
  CONNECT_TCP,
  CONNECT_UART
} connection_t;

int transmit_mav_msgs(connection_t con, mavlink_message_t *msg);
void* mavlink_rx_thread(void* arg);
void* mavlink_tx_thread(void* arg);

// Допустимые бодрейты прописаны в winbase.h через #define (например CBR_115200)
const int valid_baudrates[VALID_BAUDRATES_SIZE] = {110,   300,   600,    1200,   2400,
                                                   4800,  9600,  14400,  19200,  38400,
                                                   56000, 57600, 115200, 128000, 256000};

// СОЕДИНЕНИЯ
connection_t con_type = CONNECT_NO_SELECT;
// tcp
WSADATA wsa;
struct sockaddr_in server;
SOCKET client_socket;
// uart
HANDLE hSerial;

// MAVLink
uint8_t drone_id = 0;
mavlink_heartbeat_t drone_hbeat = {};
mavlink_global_position_int_t drone_pose = {};
mavlink_statustext_t drone_text = {};

// STATE MACHINE
task_t current_task = TASK_START_TASK;

int main(int argc, char *argv[]) {
  /**
   * НАСТРОЙКА ПОДКЛЮЧЕНИЯ
   */
  switch (argc) {
    case 1: {
      con_type = CONNECT_TCP;
      printf("  TCP\n-------\n");
      // Инициализация Winsock
      if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        printf("WSAStartup failed. Error: %d\n", WSAGetLastError());
        return 1;
      }
      printf("WSAStartup OK\n");
      
      // Создание сокета
      if ((client_socket = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET) {
        printf("Socket creation failed. Error: %d\n", WSAGetLastError());
        WSACleanup();
        return 1;
      }
      printf("Socket creation OK\n");

      // Настройка адреса сервера
      server.sin_family = AF_INET;
      server.sin_addr.s_addr = inet_addr(SERVER_IP);
      server.sin_port = htons(SERVER_PORT);
      printf("Connecting to %s:%d...\n", SERVER_IP, SERVER_PORT);

      // Подключение к серверу
      if (connect(client_socket, (struct sockaddr*)&server, sizeof(server)) < 0) {
        printf("Connection failed. Error: %d\n", WSAGetLastError());
        closesocket(client_socket);
        WSACleanup();
        return 1;
      }
      printf("Connection OK\n");
      break;
    }
    case 3: {
      con_type = CONNECT_UART;
      printf("  UART\n--------\n");
      // Разбираем аргументы main()
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
      hSerial = CreateFile(comport_buff, GENERIC_READ | GENERIC_WRITE, 0, 0,
                           OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
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
      break;
    }
    default:
      printf("Invalid parameters\n");
      printf("For TCP:\nardu-guided.exe\n");
      printf("For UART:\nardu-guided.exe [num of COM port] [baudrate]\n");
      return 1;
  }

  pthread_t mavlink_rx, mavlink_tx;
  pthread_create(&mavlink_rx, NULL, mavlink_rx_thread, NULL);
  pthread_create(&mavlink_tx, NULL, mavlink_tx_thread, NULL);

  /**
   * РАБОТА STATE MACHINE
   */
  printf("-- SM: START\n");
  for (;;) {
    switch (current_task) {
      case TASK_START_TASK: {
        // Проверка, что получили GLOBAL_POSITION_INT
        if (drone_pose.time_boot_ms != 0) {
          current_task = TASK_TO_GUIDED;
          printf("-- SM: TASK_TO_GUIDED\n");
        }  
        break;
      }
      case TASK_TO_GUIDED: {
        // Экстренное прекращение выполнения,
        // путем перевода дрона в ручной режим Loiter
        if (drone_hbeat.custom_mode == COPTER_MODE_LOITER) {
          current_task = TASK_DO_NOTHING;
        } else if (drone_hbeat.custom_mode == COPTER_MODE_GUIDED) {
          current_task = TASK_ARM;
          printf("-- SM: TASK_ARM\n");
        }
        break;
      }
      case TASK_ARM: {
        // Здесь и дальше защита от повторного выполнения кода
        // после перевода в ручной режим
        if (drone_hbeat.custom_mode != COPTER_MODE_GUIDED) {
          current_task = TASK_DO_NOTHING;
        } else if (drone_hbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) {
          current_task = TASK_TAKEOFF;
          printf("-- SM: TASK_TAKEOFF\n");
        }
        break;
      }
      case TASK_TAKEOFF: {
        if (drone_hbeat.custom_mode != COPTER_MODE_GUIDED) {
          current_task = TASK_DO_NOTHING;
        } else if (fabs(TAKEOFF_ALT - (drone_pose.relative_alt / 1000.)) <= TAKEOFF_ALT_DELTA) {
           current_task = TASK_FLY_TO_COORD;
          printf("-- SM: TASK_FLY_TO_COORD\n");
        }
        break;
      }
      case TASK_FLY_TO_COORD: {
        if (drone_hbeat.custom_mode != COPTER_MODE_GUIDED) {
          current_task = TASK_DO_NOTHING;
        }
        break;
      }
    }
  }
  return 0;
}

void* mavlink_rx_thread(void* arg) {
  uint8_t rx_buf [RX_BUFF_SIZE];
  DWORD rx_real_count = 0;
  mavlink_message_t rx_msg = {};
  mavlink_status_t rx_status = {};
  for (;;) {
    /**
     * Получаем данные
     */
    if (con_type == CONNECT_TCP) {
      rx_real_count = recv(client_socket, rx_buf, RX_BUFF_SIZE - 1, 0);
      if (rx_real_count == SOCKET_ERROR) {
        rx_real_count = 0;
        int rx_error = WSAGetLastError();
        printf("SOCKET_ERROR %d\n", rx_error);
        if (rx_error != WSAEWOULDBLOCK) {
          printf("Connection failed. Error: %d\n",rx_error);
        }
      } else if (rx_real_count == 0) {
        printf("Connection closed by server\n");
        break;
      }
    } else {
      ReadFile(hSerial, rx_buf, RX_BUFF_SIZE, &rx_real_count, NULL);
    }
    /**
     * Парсинг
     */
    for (int i = 0; i < rx_real_count; i++) {
      // Функция mavlink_parse_char пытается собрать пакет по одному байту, используя свой внутренний буфер
      if (mavlink_parse_char(0, rx_buf[i], &rx_msg, &rx_status)) {
        switch (rx_msg.msgid) {
          // Это сообщение отсылается каждую секунду. По нему очень удобно находить дрон
          case MAVLINK_MSG_ID_HEARTBEAT:
            // Если компонент, отправивший HEARTBEAT является автопилотом
            if (rx_msg.compid == MAV_COMP_ID_AUTOPILOT1) {
              mavlink_heartbeat_t unknown_hbeat;
              mavlink_msg_heartbeat_decode (&rx_msg, &unknown_hbeat);
              // Если автопилот Ardupilot и тип дрона - мультиротор
              if (unknown_hbeat.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA &&
                  (unknown_hbeat.type == MAV_TYPE_QUADROTOR ||
                    unknown_hbeat.type == MAV_TYPE_HEXAROTOR ||
                    unknown_hbeat.type == MAV_TYPE_OCTOROTOR)) {
                if (!drone_id) {
                  drone_id = rx_msg.sysid; // Запоминаем id дрона
                  printf("HEARTBEAT: New drone [id %d], saved\n", rx_msg.sysid);
                } else if (rx_msg.sysid == drone_id) {
                  drone_hbeat = unknown_hbeat;
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
              mavlink_msg_global_position_int_decode(&rx_msg, &drone_pose);
              // Внимательнее с высотой, тут есть от точки запуска и от WGS84
              printf("POSITION lat:%d, lon:%d, alt:%f\n", drone_pose.lat, drone_pose.lon, drone_pose.relative_alt / 1000.);
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
      }
    }
    Sleep(1000);
  }
  return 0;
}

void* mavlink_tx_thread(void* arg) {
  uint8_t this_id = 254; // 255 - стандарт для наземных станций, мы будем использовать другой на всякий случай
  // TODO: target_lat, target_lon, target_alt должны откуда-то обновляться
  int32_t target_lat = 584363792; // degE7
  int32_t target_lon = 309819603;
  float target_alt = 15; // m
  mavlink_message_t tx_msg = {};
  for (;;) {
    if (drone_id) {
      switch (current_task) {
        // 1. Начало выполнения задачи, запрашиваем сообщения
        case TASK_START_TASK: {
          printf("  SEND: Req MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
          mavlink_msg_command_long_pack(this_id, MAV_COMP_ID_MISSIONPLANNER, &tx_msg,
                                        drone_id, MAV_COMP_ID_AUTOPILOT1, MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                        MAVLINK_MSG_ID_GLOBAL_POSITION_INT,  // Что запрашиваем
                                        1000000,  // Интервал в микросекундах
                                        0, 0, 0, 0, 0);
          transmit_mav_msgs(con_type, &tx_msg);
          Sleep(PERIOD_TX_START_TASK_MS);
          break;
        }
        // 2. Переводим в режим Guided
        case TASK_TO_GUIDED: {
          printf("  SEND: COPTER_MODE_GUIDED\n");
          mavlink_msg_command_long_pack(this_id, MAV_COMP_ID_MISSIONPLANNER, &tx_msg,
                                        drone_id, MAV_COMP_ID_AUTOPILOT1,
                                        MAV_CMD_DO_SET_MODE , 0,
                                        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, COPTER_MODE_GUIDED,
                                        0, 0, 0, 0, 0);
          transmit_mav_msgs(con_type, &tx_msg);
          Sleep(PERIOD_TX_TO_GUIDED_MS);
          break;
        }
        // 3. Армим дрон
        case TASK_ARM: {
          printf("  SEND: ARM\n");
          mavlink_msg_command_long_pack(this_id, MAV_COMP_ID_MISSIONPLANNER, &tx_msg,
                                        drone_id, MAV_COMP_ID_AUTOPILOT1,
                                        MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                        1, 0, 0, 0, 0, 0, 0); // 1 - ARM
          transmit_mav_msgs(con_type, &tx_msg);
          Sleep(PERIOD_TX_ARM_MS);
          break;
        }
        // 4. Отправляем команды на взлет
        case TASK_TAKEOFF: {
          printf("  SEND: TAKEOFF\n");
          mavlink_msg_command_long_pack(this_id, MAV_COMP_ID_MISSIONPLANNER, &tx_msg,
                                        drone_id, MAV_COMP_ID_AUTOPILOT1,
                                        MAV_CMD_NAV_TAKEOFF, 0,
                                        0, 0, 0, 0, 0, 0, TAKEOFF_ALT);
          transmit_mav_msgs(con_type, &tx_msg);
          Sleep(PERIOD_TX_TAKEOFF_MS);
          break;
        }
        // 5. Отправляем координаты
        case TASK_FLY_TO_COORD: {
          printf ("  SEND: Pos global lat:%d, lon:%d, alt:%f\n", target_lat, target_lon, target_alt);
          mavlink_msg_set_position_target_global_int_pack(
            this_id, MAV_COMP_ID_MISSIONPLANNER, &tx_msg,
            0,  // time_boot_ms скорее всего можно оставить 0
            drone_id, MAV_COMP_ID_AUTOPILOT1,
            MAV_FRAME_GLOBAL_RELATIVE_ALT,  // Возможно другой фрейм
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
          transmit_mav_msgs(con_type, &tx_msg);
          Sleep(PERIOD_TX_FLY_TO_COORD_MS);
          break;
        }
      }
    }
  }
  return 0;
}

int transmit_mav_msgs(connection_t con, mavlink_message_t *msg) {
  uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t tx_len = mavlink_msg_to_send_buffer(tx_buf, msg);
  if (con == CONNECT_TCP) {
    if (send(client_socket, tx_buf, tx_len, 0) == SOCKET_ERROR) {
      return WSAGetLastError();
    }
  } else {
    WriteFile(hSerial, tx_buf, tx_len, NULL, NULL);
  }
  return 0;
}

