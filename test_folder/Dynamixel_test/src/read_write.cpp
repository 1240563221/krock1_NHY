  /*
  * read_write.c
  *
  *  Created on: 2016. 5. 16.
  *      Author: Leon Ryu Woon Jung
  */

  //
  // *********     Read and Write Example      *********
  //
  //
  // Available DXL model on this example : All models using Protocol 1.0
  // This example is designed for using a Dynamixel MX-28, and an USB2DYNAMIXEL.
  // To use another Dynamixel model, such as X series, see their details in E-Manual(support.robotis.com) and edit below "#define"d variables yourself.
  // Be sure that Dynamixel MX properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 1000000 [1M])
  //

  #ifdef __linux__
  #include <unistd.h>
  #include <fcntl.h>
  #include <termios.h>
  #elif defined(_WIN32) || defined(_WIN64)
  #include <conio.h>
  #endif

  #include <stdlib.h>
  #include <stdio.h>
  #include "dynamixel_sdk.h"                                   // Uses DYNAMIXEL SDK library

  // Control table address
  #define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
  #define ADDR_MX_GOAL_POSITION           30
  #define ADDR_MX_PRESENT_POSITION        36

  // Protocol version
  #define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

  // Default setting

  #define DXL_ID                          1                   // Dynamixel ID: 1
  #define BAUDRATE                        57600
  #define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                              // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

  #define TORQUE_ENABLE                   1                   // Value for enabling the torque
  #define TORQUE_DISABLE                  0                   // Value for disabling the torque
  #define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
  #define DXL_MAXIMUM_POSITION_VALUE      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
  #define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

  #define ESC_ASCII_VALUE                 0x1b

  int getch()
  {
  #ifdef __linux__
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
  #elif defined(_WIN32) || defined(_WIN64)
    return _getch();
  #endif
  }

  int kbhit(void)
  {
  #ifdef __linux__
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
      ungetc(ch, stdin);
      return 1;
    }

    return 0;
  #elif defined(_WIN32) || defined(_WIN64)
    return _kbhit();
  #endif
  }

  int main()
  {

    dynamixel::PortHandler *portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint16_t dxl_present_position = 0;              // Present position
    int a,b;
        // Open port
    // if (openPort(port_num))
    if (portHandler_->openPort())
    {
      printf("Succeeded to open the port!\n");
    }
    else
    {
      printf("Failed to open the port!\n");
      printf("Press any key to terminate...\n");
      getch();
      return 0;
    }

    // Set port baudrate
    // if (setBaudRate(port_num, BAUDRATE))
    if (portHandler_->setBaudRate(BAUDRATE))
    {
      printf("Succeeded to change the baudrate!\n");
    }
    else
    {
      printf("Failed to change the baudrate!\n");
      printf("Press any key to terminate...\n");
      getch();
      return 0;
    }

    // Enable DXL Torque
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, 2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, 3, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
#ifdef  MY_DEBUG
      printf("---------111-1-----------\n");
#endif
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler_->getRxPacketError(dxl_comm_result));
#ifdef  MY_DEBUG
      printf("---------111-2-----------\n");
#endif
    }
    else
    {
      printf("Dynamixel has been successfully connected \n");
    }

    uint8_t my_id=0;
    while(1)
    {
        printf("Please input a  id number(1-3):\n");
        scanf("%d",&my_id);
        if(my_id<1 || my_id>3)
        {
            printf("The id number out of limit! Try again!\n");
            continue;
        }

        printf("Please input a number(100-4000):\n");
        scanf("%d",&a);
        if(a<100 || a>4000)
        {
            printf("The number out of limit!Please input again!\n");
            continue;
        }
        else
        {
            packetHandler_->write2ByteTxRx(portHandler_, my_id, ADDR_MX_GOAL_POSITION, a, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
        #ifdef  MY_DEBUG
                printf("---------222-1-----------\n");
        #endif
            }
            else if (dxl_error != 0)
            {
                printf("%s\n", packetHandler_->getRxPacketError(dxl_comm_result));
        #ifdef  MY_DEBUG
                printf("---------222-2-----------\n");
        #endif
            }
        }
    }


//      // Initialize PortHandler instance
//   // Set the port path
//   // Get methods and members of PortHandlerLinux or PortHandlerWindows
//   dynamixel::PortHandler *portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);

//   // Initialize PacketHandler instance
//   // Set the protocol version
//   // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
//   dynamixel::PacketHandler *packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
//     int index = 0;
//     int dxl_comm_result = COMM_TX_FAIL;             // Communication result
//     int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };         // Goal position

//     uint8_t dxl_error = 0;                          // Dynamixel error
//     uint16_t dxl_present_position = 0;              // Present position

//     // Open port
//     // if (openPort(port_num))
//     if (portHandler_->openPort())
//     {
//       printf("Succeeded to open the port!\n");
//     }
//     else
//     {
//       printf("Failed to open the port!\n");
//       printf("Press any key to terminate...\n");
//       getch();
//       return 0;
//     }

//     // Set port baudrate
//     // if (setBaudRate(port_num, BAUDRATE))
//     if (portHandler_->setBaudRate(BAUDRATE))
//     {
//       printf("Succeeded to change the baudrate!\n");
//     }
//     else
//     {
//       printf("Failed to change the baudrate!\n");
//       printf("Press any key to terminate...\n");
//       getch();
//       return 0;
//     }

//     // Enable DXL Torque
//     dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
//     if (dxl_comm_result != COMM_SUCCESS)
//     {
//       printf("%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
// #ifdef  MY_DEBUG
//       printf("---------111-1-----------\n");
// #endif
//     }
//     else if (dxl_error != 0)
//     {
//         printf("%s\n", packetHandler_->getRxPacketError(dxl_comm_result));
// #ifdef  MY_DEBUG
//       printf("---------111-2-----------\n");
// #endif
//     }
//     else
//     {
//       printf("Dynamixel has been successfully connected \n");
//     }

//     while (1)
//     {
//       printf("Press any key to continue! (or press ESC to quit!)\n");
//       if (getch() == ESC_ASCII_VALUE)
//         break;

//       // Write goal position
//       packetHandler_->write2ByteTxRx(portHandler_, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
//       if (dxl_comm_result != COMM_SUCCESS)
//       {
//         printf("%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
// #ifdef  MY_DEBUG
//         printf("---------222-1-----------\n");
// #endif
//       }
//       else if (dxl_error != 0)
//       {
//         printf("%s\n", packetHandler_->getRxPacketError(dxl_comm_result));
// #ifdef  MY_DEBUG
//         printf("---------222-2-----------\n");
// #endif
//       }

//       do
//       {
//         // Read present position
//         dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
//         if (dxl_comm_result != COMM_SUCCESS)
//         {
//             printf("%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
// #ifdef  MY_DEBUG
//             printf("---------333-1-----------\n");
// #endif
//         }
//         else if (dxl_error != 0)
//         {
//             printf("%s\n", packetHandler_->getRxPacketError(dxl_comm_result));
// #ifdef  MY_DEBUG
//             printf("---------333-2-----------\n");
// #endif
//         }

//         printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, dxl_goal_position[index], dxl_present_position);

//       } while ((abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

//       // Change goal position
//       if (index == 0)
//       {
//         index = 1;
//       }
//       else
//       {
//         index = 0;
//       }
//     }

    // Disable Dynamixel Torque
    packetHandler_->write1ByteTxRx(portHandler_, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler_->getRxPacketError(dxl_comm_result));
    }

    // Close port
    portHandler_->closePort();

    return 0;
  }