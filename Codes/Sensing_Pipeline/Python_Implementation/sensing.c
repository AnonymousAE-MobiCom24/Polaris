#include <stdio.h>
#include <signal.h>
#include <math.h>
#include <stdlib.h>
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>   /* For open(), creat() */
#include <termios.h>  /* POSIX terminal control definitions */
#include <string.h>   /* For memset */
#include <stdbool.h>


#define COM_PORT "/dev/rplidar"  // Replace with your actual COM port

// 全局标志
int interrupted = 0;

// Signal handler
void interruptHandler(int sig) {
    printf("Interrupted.\n");
    interrupted = 1;
}


// 定义结构体
struct DynamicArray
{
    int length;
    double *data;
};


struct Point3D 
{
    double x;
    double y;
    double z;
    double total;
};

struct SensorArray
{
    struct Point3D *data;
    size_t size;
    size_t capacity; 
};

void initDynamicArray(struct SensorArray *arr, size_t initialCapacity) 
{
    arr->data = malloc(initialCapacity * sizeof(struct Point3D));
    if (arr->data == NULL) {
        fprintf(stderr, "Memory allocation failed.\n");
        exit(EXIT_FAILURE);
    }
    arr->size = 0;
    arr->capacity = initialCapacity;
}

void appendToDynamicArray(struct SensorArray *arr, double x, double y, double z, double total) 
{
    if (arr->size == arr->capacity) {
        arr->capacity *= 2;
        arr->data = realloc(arr->data, arr->capacity * sizeof(struct Point3D));
        if (arr->data == NULL) {
            fprintf(stderr, "Memory allocation failed.\n");
            exit(EXIT_FAILURE);
        }
    }
    arr->data[arr->size].x = x;
    arr->data[arr->size].y = y;
    arr->data[arr->size].z = z;
    arr->data[arr->size].total = total;
    arr->size++;
}

void freeDynamicArray(struct SensorArray *arr) {
    free(arr->data);
    arr->data = NULL;
    arr->size = 0;
    arr->capacity = 0;
}



#define WND 5
#define WND_D 3

double gaussian(double x, double pos, double wid) 
{
    return exp(-pow((x - pos) / (0.60056120439323 * wid), 2));
}

void normalize(double *array, int size) 
{
    double sum = 0.0;
    for (int i = 0; i < size; i++)
        sum += array[i];
    for (int i = 0; i < size; i++)
        array[i] /= sum;
}


int main()
{
    signal(SIGINT, interruptHandler);
    int serial_fd = open(COM_PORT, O_RDWR | O_NOCTTY | O_NDELAY);


    if (serial_fd == -1)
    {
        // Handle error opening the serial port
        perror("Error opening COM port\n");
        return 1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(serial_fd, &tty) != 0)
    {
        // Handle error getting terminal attributes
        perror("Error getting serial port state\n");
        close(serial_fd);
        return 1;
    }
    cfsetospeed(&tty, B115200);  // Adjust baud rate as needed
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication
    tty.c_cflag &= ~CSIZE;  // Clear all the size bits
    tty.c_cflag |= CS8;     // 8 bits per byte
    // tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0)
    {
        // Handle error setting terminal attributes
        perror("Error setting serial port state\n");
        close(serial_fd);
        return 1;
    }


    double SmoothVector_r[WND];
    double SmoothVector_d[WND_D];

    for (int i = 0; i < WND; i++) 
    {
        double x_val = i + 1;
        SmoothVector_r[i] = gaussian(x_val, WND / 2.0, WND / 2.0);
    }
    normalize(SmoothVector_r, WND);

    for (int i = 0; i < WND_D; i++) 
    {
        double x_val = i + 1;
        SmoothVector_d[i] = gaussian(x_val, WND_D / 2.0, WND_D / 2.0);
    }
    normalize(SmoothVector_d, WND_D);



    //
    const int num = 9;
    double slope_thr[num];
    for (int i = 0; i < num; ++i)
        slope_thr[i] = 10000;

    double LastRAW[num];
    for (int i = 0; i < num; ++i)
        LastRAW[i] = 0.0;

    int S_flag[num];
    for (int i = 0; i < num; ++i)
        S_flag[i] = 0;

    double delta_thrd[num];
    for (int i = 0; i < num; ++i)
        delta_thrd[i] = 12;

    double amp_thrd[num];
    for (int i = 0; i < num; ++i)
        amp_thrd[i] = 4;

    bool AuxiliaryFlag[num];
    for (int i = 0; i < num; ++i)
        AuxiliaryFlag[i] = true;

    struct DynamicArray slope_list[num];
    struct DynamicArray raw_list[num];
    struct SensorArray sensors[num];
    struct SensorArray smooth_sensors[num];
    struct SensorArray derivative_sensors[num];
    struct SensorArray smooth_derivative_sensors[num];

    for (int i = 0; i < num; ++i) 
    {
        slope_list[i].length = 0; // 初始化长度为0
        slope_list[i].data = NULL; // 初始化数据指针为NULL
    }
    for (int i = 0; i < num; ++i)
    {
        initDynamicArray(&sensors[i], 1000); 
        initDynamicArray(&smooth_sensors[i], 1000);
        initDynamicArray(&derivative_sensors[i], 1000);
        initDynamicArray(&smooth_derivative_sensors[i], 1000);
    }


    int cnt = 0;
    int cnt_tol = 0;
    int first = 0;
    int no = 0;

    double sensor_tol_data[num];
    double sensor_x_data[num];
    double sensor_y_data[num];
    double sensor_z_data[num];

    while (!interrupted)
    {
        int flag = 1;
        ssize_t bytesRead;
        char buffer[3*sizeof(float) *num];
        // printf("%d", sizeof(float));
        bytesRead = read(serial_fd, buffer, sizeof(buffer));
        // printf("cnt_tol: %d\n", cnt_tol);
            
        // if (cnt_tol > 100)
        // {
        // float* x = (float*)&buffer[0];
        // printf("%lf", *x);
        // printf("%d\n", bytesRead);
        if (bytesRead>0)
        {
            
            for (ssize_t i = 0; i < bytesRead / sizeof(float)/3; ++i)
            {
                // Interpret the bytes as a float
                float* x = (float*)&buffer[3 * i * sizeof(float)];
                float* y = (float*)&buffer[3 * i * sizeof(float) + sizeof(float)];
                float* z = (float*)&buffer[3 * i * sizeof(float) + sizeof(float) * 2];
                // print x, y, z
                // printf("Received Float: %f, %f, %f, of sensor %ld \n", *x, *y, *z, i+1);
                // continue;
                // if(*x > 10000 || *y > 10000 || *z > 10000)
                //     flag = 0;
                double total = sqrt(pow(*x, 2) + pow(*y, 2) + pow(*z, 2));
                sensor_x_data[i] = *x;
                sensor_y_data[i] = *y;
                sensor_z_data[i] = *z;
                sensor_tol_data[i] = total;
                // printf("%f\n", total);
                // appendToDynamicArray(&sensors[i], *x, *y, *z, total);
            }
            if(first == 0)
            {
                for (int i = 0; i < num; ++i)
                {
                    if(isnan(sensor_tol_data[i]) || sensor_tol_data[i] > 4000)
                        flag = 0;
                }
            }
            if(flag)
            {
                if (first == 0)
                {
                    first = 1;
                    printf("Start\n");
                }
                for (int i = 0; i < num; ++i)
                {
                    appendToDynamicArray(&sensors[i], sensor_x_data[i], sensor_y_data[i], sensor_z_data[i], sensor_tol_data[i]);
                }
            }
            else
            {
                printf("Invalid data\n");
                continue;
            }
              
            // printf("\n");
        
            // printf("size: %d\n", sensors[0].size);

            // peak detection function
            // 1. smooth the data
            if (cnt < WND)
            {
                for(int i = 0; i < num; ++i)
                {
                    double x = sensors[i].data[cnt].x;
                    double y = sensors[i].data[cnt].y;
                    double z = sensors[i].data[cnt].z;
                    double tol = sensors[i].data[cnt].total;
                    appendToDynamicArray(&smooth_sensors[i], x, y, z, tol);
                    appendToDynamicArray(&derivative_sensors[i], 0.0, 0.0, 0.0, 0.0);
                    appendToDynamicArray(&smooth_derivative_sensors[i], 0.0, 0.0, 0.0, 0.0);
                }
                cnt += 1;
                continue;
            }

            // 1. smooth the raw data
            for(int i = 0; i < num; i++)
            {
                double sum_x = 0.0;
                double sum_y = 0.0;
                double sum_z = 0.0;
                double sum_tol = 0.0;
                for (int j = 0; j < WND; j++) {
                    sum_x += SmoothVector_r[j] * sensors[i].data[cnt - WND + j].x;
                    sum_y += SmoothVector_r[j] * sensors[i].data[cnt - WND + j].y;
                    sum_z += SmoothVector_r[j] * sensors[i].data[cnt - WND + j].z;
                    sum_tol += SmoothVector_r[j] * sensors[i].data[cnt - WND + j].total;
                }
                // printf("sum_tol: %f\n", sum_tol);
                appendToDynamicArray(&smooth_sensors[i], sum_x, sum_y, sum_z, sum_tol);
            }

            // 2. calculate the derivative
            for(int i = 0; i < num; i++)
            {
                double last_point = smooth_sensors[i].data[smooth_sensors[i].size-1].total;
                double first_point = smooth_sensors[i].data[smooth_sensors[i].size-2].total;
                // printf("last_point: %f\n", last_point);
                // printf("first_point: %f\n", first_point);
                double derivative = (last_point - first_point) / 2.0;
                // printf("derivative: %f\n", derivative);
                appendToDynamicArray(&derivative_sensors[i], 0.0, 0.0, 0.0, derivative);
            }

            // 3. smooth the derivative
            for(int i = 0; i < num; i++)
            {
                double sum_x = 0.0;
                double sum_y = 0.0;
                double sum_z = 0.0;
                double sum_tol = 0.0;
                for (int j = 0; j < WND_D; j++) {
                    // sum_x += SmoothVector_d[j] * derivative_sensors[i].data[cnt - WND_D + j].x;
                    // sum_y += SmoothVector_d[j] * derivative_sensors[i].data[cnt - WND_D + j].y;
                    // sum_z += SmoothVector_d[j] * derivative_sensors[i].data[cnt - WND_D + j].z;
                    sum_tol += SmoothVector_d[j] * derivative_sensors[i].data[cnt - WND_D + j].total;
                }
                appendToDynamicArray(&smooth_derivative_sensors[i], sum_x, sum_y, sum_z, sum_tol);
            }

            // 4. peak detection
            for(int i = 0; i < num; i++)
            {
                S_flag[i] = 0;

                if(slope_list[i].length == 10)
                {
                    if(AuxiliaryFlag[i])
                    {
                        double tmp = 0.0;
                        for(int j = 1; j < slope_list[i].length; j++)
                            tmp += fabs(slope_list[i].data[j]);
                        slope_thr[i] = amp_thrd[i] * (tmp / (slope_list[i].length - 1));
                        AuxiliaryFlag[i] = false;
                        printf("Sensor %d: slope_thr = %f\n", i + 1, slope_thr[i]);
                    }
                }
                // Positive peak
                double smoothed_1 = smooth_derivative_sensors[i].data[smooth_derivative_sensors[i].size-1].total;
                double smoothed_2 = smooth_derivative_sensors[i].data[smooth_derivative_sensors[i].size-2].total;
                if(smoothed_1 <= 0.0 && smoothed_2 >= 0.0)
                {
                    double slope = smoothed_1 - smoothed_2;
                    slope_list[i].length++;
                    slope_list[i].data = (double *)realloc(slope_list[i].data, slope_list[i].length * sizeof(double));
                    if (slope_list[i].data == NULL) {
                        //
                        fprintf(stderr, "Memory allocation failed.\n");
                        // return 1;
                    }
                    slope_list[i].data[slope_list[i].length - 1] = slope;
                    // printf("%d", slope_list[i].length);
                    int raw_index = cnt - WND;
                    double raw = smooth_sensors[i].data[raw_index].total;
                    if (slope <= -slope_thr[i])
                    {
                        if(fabs(LastRAW[i]) <= 1e-6)
                        {
                            S_flag[i] = 1;
                            no = no + 1;
                            // printf("%d\n", raw_index);
                            // printf("%f\n", slope);
                            printf("Sensor %d: No. %d detect a magnet\n", i + 1, no);
                            // printf("Sensor %d: raw data: %f\n", i + 1, raw);
                                // print x, y, z data
                            printf("Sensor %d: x: %f, y: %f, z: %f\n", i + 1, smooth_sensors[i].data[raw_index].x, smooth_sensors[i].data[raw_index].y, smooth_sensors[i].data[raw_index].z);

                        }
                        else
                        {
                            if(raw - LastRAW[i] >= delta_thrd[i])
                            {
                                S_flag[i] = 1;
                                no = no + 1;
                                // printf("%d\n", raw_index);
                                // printf("%f\n", slope);
                                printf("Sensor %d: No. %d detect a magnet\n", i + 1, no);
                                // print the raw data
                                // printf("Sensor %d: raw data: %f\n", i + 1, raw);
                                // print x, y, z data
                                printf("Sensor %d: x: %f, y: %f, z: %f\n", i + 1, smooth_sensors[i].data[raw_index].x, smooth_sensors[i].data[raw_index].y, smooth_sensors[i].data[raw_index].z);
                            } 
                        }
                    }
                    else
                    {
                        for(int i = 0; i < num; i++)
                        {
                            // LastRAW[i] = (np.array(sz[i][raw_index - 15: raw_index-10])).mean()
                            double s = 0.0;
                            int interval = 15;
                            for (int j = 0; j < interval; j++)
                                s += smooth_sensors[i].data[raw_index-40+j].total;
                            LastRAW[i] = s / 15.0;
                        }
                    }
                        
                }
            }
        
        // ddtw matching function
        

        // tag reconstruction function

            cnt ++;
        }
    }
    //     cnt_tol ++;
    // }
    
    // record the smoothed data to csv
    // FILE *fp;
    // fp = fopen("data.csv", "w");
    // for (int i = 0; i < smooth_sensors[0].size; ++i)
    // {
    //     // fprintf(fp, "%f, %f, %f\n", smooth_sensors[0].data[i].total, smooth_sensors[1].data[i].total, smooth_sensors[2].data[i].total);
    //     fprintf(fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n", smooth_sensors[0].data[i].total, smooth_sensors[1].data[i].total, smooth_sensors[2].data[i].total, sensors[3].data[i].total, sensors[4].data[i].total, sensors[5].data[i].total, sensors[6].data[i].total, sensors[7].data[i].total, sensors[8].data[i].total);
    // }

    // fp = fopen("data_x.csv", "w");
    // for (int i = 0; i < smooth_sensors[0].size; ++i)
    // {
    //     // fprintf(fp, "%f, %f, %f\n", smooth_sensors[0].data[i].total, smooth_sensors[1].data[i].total, smooth_sensors[2].data[i].total);
    //     fprintf(fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n", smooth_sensors[0].data[i].x, smooth_sensors[1].data[i].x, smooth_sensors[2].data[i].x, sensors[3].data[i].x, sensors[4].data[i].x, sensors[5].data[i].x, sensors[6].data[i].x, sensors[7].data[i].x, sensors[8].data[i].x);
    // }

    // fp = fopen("data_y.csv", "w");
    // for (int i = 0; i < smooth_sensors[0].size; ++i)
    // {
    //     // fprintf(fp, "%f, %f, %f\n", smooth_sensors[0].data[i].total, smooth_sensors[1].data[i].total, smooth_sensors[2].data[i].total);
    //     fprintf(fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n", smooth_sensors[0].data[i].y, smooth_sensors[1].data[i].y, smooth_sensors[2].data[i].y, sensors[3].data[i].y, sensors[4].data[i].y, sensors[5].data[i].y, sensors[6].data[i].y, sensors[7].data[i].y, sensors[8].data[i].y);
    // }

    // fp = fopen("data_z.csv", "w");
    // for (int i = 0; i < smooth_sensors[0].size; ++i)
    // {
    //     // fprintf(fp, "%f, %f, %f\n", smooth_sensors[0].data[i].total, smooth_sensors[1].data[i].total, smooth_sensors[2].data[i].total);
    //     fprintf(fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n", smooth_sensors[0].data[i].z, smooth_sensors[1].data[i].z, smooth_sensors[2].data[i].z, sensors[3].data[i].z, sensors[4].data[i].z, sensors[5].data[i].z, sensors[6].data[i].z, sensors[7].data[i].z, sensors[8].data[i].z);
    // }




    FILE *fp2;
    fp2 = fopen("derivative_data.csv", "w");
    for (int i = 0; i < smooth_derivative_sensors[0].size; ++i)
    {
        // fprintf(fp2, "%f, %f, %f\n", smooth_derivative_sensors[0].data[i].total, smooth_derivative_sensors[1].data[i].total, smooth_derivative_sensors[2].data[i].total);
        fprintf(fp2, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n", smooth_derivative_sensors[0].data[i].total, smooth_derivative_sensors[1].data[i].total, smooth_derivative_sensors[2].data[i].total, smooth_derivative_sensors[3].data[i].total, smooth_derivative_sensors[4].data[i].total, smooth_derivative_sensors[5].data[i].total, smooth_derivative_sensors[6].data[i].total, smooth_derivative_sensors[7].data[i].total, smooth_derivative_sensors[8].data[i].total);   
    }

    close(serial_fd);
    // free memory
    for (int i = 0; i < num; ++i)
    {
        freeDynamicArray(&sensors[i]);
        freeDynamicArray(&smooth_sensors[i]);
        freeDynamicArray(&derivative_sensors[i]);
        freeDynamicArray(&smooth_derivative_sensors[i]);
    }
    
    printf("Existed");
    return 0;
}
