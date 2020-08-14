#define CONFIGURE_INIT
#define RTEMS_NETWORKING
#include <rtems.h>
#include <bsp.h>
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
//#define CONFIGURE_MICROSECONDS_PER_TICK 200
#define CONFIGURE_MAXIMUM_TASKS             5
#define CONFIGURE_MAXIMUM_SEMAPHORES        10
#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES    10
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 10
#define CONFIGURE_MAXIMUM_DRIVERS 5
#define CONFIGURE_NUMBER_OF_TERMIOS_PORTS 5
#define CONFIGURE_INIT_TASK_NAME rtems_build_name( 'D', 'E', 'M', 'O' )
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_INIT_TASK_STACK_SIZE    (RTEMS_MINIMUM_STACK_SIZE * 2)
#define CONFIGURE_EXTRA_TASK_STACKS       RTEMS_MINIMUM_STACK_SIZE
#define CONFIGURE_INIT_TASK_ATTRIBUTES RTEMS_FLOATING_POINT
#include <rtems/confdefs.h>

#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>
#include <cstdio>
#include <fstream>
#include <pthread.h>
#include "ShiftedGridPlanner.h"
#include "LinearInterpolationPathExtractor.h"
#include "Graph.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <rtems/rtems_bsdnet.h>
#include <arpa/inet.h>
#include <rtems/error.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <rtems/rtc.h>
#undef RTEMS_USE_LOOPBACK
#include "../networkconfig.h"

#ifndef OPT_LVL
#define OPT_LVL 2
#endif

void print_bytes(void *p, size_t len)
{
    size_t i;
    printf("(");
    for (i = 0; i < len; ++i)
        printf("%02X", ((unsigned char*)p)[i]);
    printf(")\n");
}

rtems_task Init(rtems_task_argument) {



    BSP_led_1_on();
    BSP_led_2_on();
    std::ios_base::sync_with_stdio(false);

    occan_register();
    asio4_register();
    rtc_initialize(5,0,NULL);

    printf( "Initializing network...\n" );
    rtems_bsdnet_initialize_network();
    printf( "Network initialized.\n" );
    rtems_bsdnet_show_if_stats ();
    rtems_bsdnet_show_ip_stats ();

    if ( htonl(47) == 47 ) {
        printf("Endianness: Big Endian\n");
    } else {
        printf("Endianness: Little Endian\n");
    }

    printf("\n");
    printf("sizeof(char)      %d\n", sizeof(char));
    printf("sizeof(short)     %d\n", sizeof(short));
    printf("sizeof(int)       %d\n", sizeof(int));
    printf("sizeof(int32_t)   %d\n", sizeof(int32_t));
    printf("sizeof(uint32_t)  %d\n", sizeof(uint32_t));
    printf("sizeof(long)      %d\n", sizeof(long));
    printf("sizeof(float)     %d\n", sizeof(float));
    printf("sizeof(double)    %d\n", sizeof(double));
    printf("\n");

    int s;
    static struct sockaddr_in farAddr;

    printf ("Creating socket...\n");
    s = socket (AF_INET, SOCK_STREAM, 0);
    if (s < 0)
        rtems_panic ("Can't create socket: %s", strerror (errno));
    printf ("Socket created.\n");

    farAddr.sin_family = AF_INET;
    farAddr.sin_port = htons (1234);
    farAddr.sin_addr.s_addr = inet_addr("192.168.17.128");
    memset (farAddr.sin_zero, '\0', sizeof farAddr.sin_zero);

    printf("Simulator address is [192.1658.17.128:1234].\n");
    printf ("Connecting to simulator...\n");
    if (connect (s, (struct sockaddr *)&farAddr, sizeof farAddr) < 0) {
        rtems_panic ("Can't create socket: %s", strerror (errno));
    }
    printf ("Connection established.\n");


    Position next_point, goal;
    float next_step_cost=0;

    long size;
    int32_t width = 0, height = 0, top = 0, left = 0, min = 0;
    float from_x = 0, from_y = 0, to_x = 0, to_y = 0;
    bool tof = false;

    printf ("ACK: ");
    fflush(stdout);
    char ack = 0;
    if (write(s, &ack, 1) < 0) rtems_panic ("Can't send: %s", strerror (errno));
    ack = -1;
    while (ack != 0)
        read(s, &ack, 1); //Wait for 0
    printf ("OK\n");

    BSP_led_2_off();
    std::cout << "[PLANNER]   Waiting for parameters..." << std::endl;
    read(s, &width, 4);
    read(s, &height, 4);
    std::cout << "[PLANNER]   Map size: [" << width << ", " << height << "]" << std::endl;
    size = width * height;
    std::shared_ptr<uint8_t> data(new uint8_t[size], std::default_delete<uint8_t[]>());
    int n = 0;
    while(n != size){
        n += read(s, data.get()+n, size-n); //Receive image
        printf("%d/%d\n", n, size);
    }
    std::cout << "[PLANNER]   Map received, " << n << "/"<< size << std::endl;
    read(s, &from_x, 4);
    read(s, &from_y, 4);
    std::cout << "[PLANNER]   Start position: [" << from_x << ", " << from_y << "]" << std::endl;
    read(s, &to_x, 4);
    read(s, &to_y, 4);
    std::cout << "[PLANNER]   Goal position:  [" << to_x << ", " << to_y << "]" << std::endl;
    read(s,(char *) &tof, 1);
    if (tof)
        std::cout << "[PLANNER]   Request to send cost-to-goal estimates." << std::endl;
    read(s,(char *) &min, 4); //Receive heuristic hint
    std::cout << "[PLANNER]   Heuristic hint: " << min << std::endl;
    BSP_led_2_on();

    next_point.x = from_x;
    next_point.y = from_y;
    goal.x = to_x;
    goal.y = to_y;

    std::cout << "[PLANNER]   Setting up planner..." << std::endl;
    ShiftedGridPlanner<OPT_LVL> planner{};
    LinearInterpolationPathExtractor<
            typename DFMPlanner<OPT_LVL>::Map::ElemType,
            typename DFMPlanner<OPT_LVL>::Base::Info>
            extractor(planner.get_expanded_map(), planner.get_grid());
    extractor.allow_indirect_traversals = true;
    planner.reset();
    planner.set_occupancy_threshold(1);
    planner.set_heuristic_multiplier(min);
    planner.set_map(data, width, height);
    planner.set_start(next_point);
    planner.set_goal(goal);
    std::cout << "[PLANNER]   Planner is ready." << std::endl;
    BSP_led_1_off();

    while (true) {
        std::cout << "[PLANNER]   New position: [" << next_point.x << ", " << next_point.y << "]" << std::endl;
        ack = 1;
        write(s, (char *) &ack, 1);
        Position p = next_point;
        p.x += 0.5f;
        p.y += 0.5f;
        write(s,(char *) &(p.x), 4);
        write(s,(char *) &(p.y), 4);
        write(s,(char *) &(next_step_cost), 4);
        ack = -1;
        while (ack != 1) {
            read(s,(char *) &ack, 1); //Wait for 1
        }
        read(s,(char *) &top, 4);
        read(s,(char *) &left, 4);
        read(s,(char *) &height, 4);
        read(s,(char *) &width, 4);
        std::cout << "[PLANNER]   New patch: position [" << top << ", " << left
                  << "], shape [" << width << ", " << height << "]" << std::endl;
        size = width * height;
        std::shared_ptr<uint8_t> patch(new uint8_t[size], std::default_delete<uint8_t[]>());
        read(s,(char *) patch.get(), size); //Receive image
        planner.patch_map(patch, top, left, width, height);
        read(s,(char *) &min, 4); //Receive heuristic hint
        planner.set_heuristic_multiplier(min);

        planner.step();
        extractor.extract_path();

        ack = 3;
        write(s,(char *) &ack, 1);
        auto path_size = (int) extractor.path_.size();
        write(s,(char *) &path_size, 4);
        for (const auto &pose : extractor.path_) {
            Position p = pose; //p.x+=0.5f; p.y+=0.5f;
            write(s,(char *) &(p.x), 4);
            write(s,(char *) &(p.y), 4);
        }
        for (const auto &step_cost: extractor.cost_) {
            write(s,(char *) &(step_cost), 4);
        }
        write(s,(char *) &(extractor.total_dist), 4);
        write(s,(char *) &(extractor.total_cost), 4);
        write(s,(char *) &(planner.u_time), 4);
        write(s,(char *) &(planner.p_time), 4);
        write(s,(char *) &(extractor.e_time), 4);

        if (tof) {
            ack = 4;
            write(s,(char *) &ack, 1);
            auto expanded_size = (long long) planner.map.size();
            write(s,(char *) &expanded_size, 8);
            for (auto b: planner.map.buckets) {
                for (const auto &expanded : b) {
                    const Cell &exp = expanded.first;
                    float g = std::get<0>(expanded.second);
                    float rhs = std::get<1>(expanded.second);
                    write(s,(char *) &(exp.x), 4);
                    write(s,(char *) &(exp.y), 4);
                    write(s,(char *) &(g), 4);
                    write(s,(char *) &(rhs), 4);
                }
            }
        }
        auto prev_point = next_point;
        for (unsigned int i = 1; i < extractor.path_.size(); i++) {
            next_point = {extractor.path_[i].x, extractor.path_[i].y};
            next_step_cost = extractor.cost_[i - 1];
            if (Cell(next_point).distance(Cell(prev_point)) > 5)
                break;
        }
        if (next_point == goal)
            break; //Goal reached
        planner.set_start(next_point);
    }

    ack = 2;
    write(s,(char *) &ack, 1);
    ack = -1;
    while (ack != 2) {
        read(s,(char *) &ack, 1); //Wait for 2
    }

    printf("####################### END ######################\n");
    while(1);

}