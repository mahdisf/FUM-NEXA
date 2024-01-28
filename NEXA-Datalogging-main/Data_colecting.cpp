#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <vector>
#include <sys/time.h>
#include <ctime>

#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>

// MPU9250 register addresses
#define MPU9250_ADDRESS 0x68
#define WHO_AM_I 0x75

const int ACCEL_XOUT_H = 0x3B;
const int gyro_XOUT_H = 0x43;
// Mutex to protect concurrent access to shared resources
std::mutex mutex;

using namespace std;

// Motor IDs
int motor_ids[2] = {0x1, 0x2};
int CAN_Socket;

// Counter variables
int enc1_counter = 0;
int enc2_counter = 0;
int mot1_counter = 0;
int mot2_counter = 0;
int imu3_counter = 0;
int imu4_counter = 0;
int imu5_counter = 0;

// Time variables
struct timeval time_now
{
};
time_t time_start;
time_t time_n;
time_t time_new;

// Sensor data variables
float back_ax = 0, back_ay = 0, back_az = 0, back_gx = 0, back_gy = 0, back_gz = 0;
float ax3 = 0, ay3 = 0, az3 = 0, gx3 = 0, gy3 = 0, gz3 = 0, ax4 = 0, ay4 = 0, az4 = 0, gx4 = 0, gy4 = 0, gz4 = 0, ax5 = 0, ay5 = 0, az5 = 0, gx5 = 0, gy5 = 0, gz5 = 0, enc1 = 0, enc2 = 0, mp1 = 0, mv1 = 0, mt1 = 0, mp2 = 0, mv2 = 0, mt2 = 0;

/////////////////////////////////////////////////////////////////////////////////
// Converts a float to an unsigned int, given range and number of bits
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// Converts an unsigned int to float, given range and number of bits
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/// Decodes the motor frame from a CAN reply message
void decodeMotorFrame(const unsigned char *CAN_reply_msg, float *response)
{
    // Unpack ints from CAN buffer
    int id = CAN_reply_msg[0];
    int p_int = (CAN_reply_msg[1] << 8) | CAN_reply_msg[2];
    int v_int = (CAN_reply_msg[3] << 4) | (CAN_reply_msg[4] >> 4);
    int i_int = ((CAN_reply_msg[4] & 0xF) << 8) | CAN_reply_msg[5];

    // Convert unsigned ints to floats
    float P = uint_to_float(p_int, -12.5f, 12.5f, 16);
    float V = uint_to_float(v_int, -50.0f, 50.0f, 12);
    float T = uint_to_float(i_int, -65.0f, 65.0f, 12);

    // Assign the results to the motor_response array
    response[0] = id;
    response[1] = P;
    response[2] = V;
    response[3] = T;
    response[4] = 0;
    response[5] = 1; // Flag indicating successful decoding
}

// Decodes sensor data from a CAN frame
void decodeSensorData(const can_frame &frame, float *decoded_response)
{
    // Encoder 1 decode
    if (frame.can_id == 1092)
    {
        decoded_response[0] = frame.can_id;
        decoded_response[1] = (float)((frame.data[1] * 256) + frame.data[0]);
        decoded_response[2] = 0.0;
        decoded_response[3] = 0.0;
        decoded_response[4] = 0.0;
        enc1_counter++;
        decoded_response[5] = 1; // flag
    }
    ///////////encoder2 decode
    if (frame.can_id == 1093)
    {
        decoded_response[0] = frame.can_id;
        decoded_response[1] = (float)((frame.data[1] * 256) + frame.data[0]);
        ;
        decoded_response[2] = 0.0;
        decoded_response[3] = 0.0;
        decoded_response[4] = 0;
        enc2_counter++;
        decoded_response[5] = 1; // flag
    }
    /////////////////motor decode
    if (frame.can_id == 0)
    {
        decodeMotorFrame(frame.data, decoded_response);
    }

    // //////////////imu
    if (frame.can_id == 3 | frame.can_id == 4 | frame.can_id == 5)
    {
        if (frame.can_dlc == 7)
        { /// DLC7-------> GX GY GZ
            int gx = (float)((frame.data[0] * 256) + frame.data[1]);
            int gy = (float)((frame.data[2] * 256) + frame.data[3]);
            int gz = (float)((frame.data[4] * 256) + frame.data[5]);
            int sign = (int)(frame.data[6]);

            decoded_response[0] = frame.can_id;
            if (sign == 1)
            {
                decoded_response[1] = -gx / 131.0;
                decoded_response[2] = gy / 131.0;
                decoded_response[3] = gz / 131.0;
            }
            else if (sign == 2)
            {
                decoded_response[1] = gx / 131.0;
                decoded_response[2] = -gy / 131.0;
                decoded_response[3] = gz / 131.0;
            }
            else if (sign == 3)
            {
                decoded_response[1] = -gx / 131.0;
                decoded_response[2] = -gy / 131.0;
                decoded_response[3] = gz / 131.0;
            }
            else if (sign == 4)
            {
                decoded_response[1] = gx / 131.0;
                decoded_response[2] = gy / 131.0;
                decoded_response[3] = -gz / 131.0;
            }
            else if (sign == 5)
            {
                decoded_response[1] = -gx / 131.0;
                decoded_response[2] = gy / 131.0;
                decoded_response[3] = -gz / 131.0;
            }
            else if (sign == 6)
            {
                decoded_response[1] = gx / 131.0;
                decoded_response[2] = -gy / 131.0;
                decoded_response[3] = -gz / 131.0;
            }
            else if (sign == 7)
            {
                decoded_response[1] = -gx / 131.0;
                decoded_response[2] = -gy / 131.0;
                decoded_response[3] = -gz / 131.0;
            }
            else if (sign == 0)
            {
                decoded_response[1] = gx / 131.0;
                decoded_response[2] = gy / 131.0;
                decoded_response[3] = gz / 131.0;
            }
            decoded_response[4] = frame.can_dlc;
            decoded_response[5] = 1;
        }
        else if (frame.can_dlc == 8)
        { /// DLC=8---------->   ACC X   &    ACC Y
            int ax = (float)((frame.data[0] << 8) | frame.data[1]);
            int ay = (float)((frame.data[2] << 8) | frame.data[3]);
            int az = (float)((frame.data[4] << 8) | frame.data[5]);
            int sign = (int)(frame.data[6]);
            ;

            decoded_response[0] = frame.can_id;
            if (sign == 1)
            {
                decoded_response[1] = -ax / 16384.0;
                decoded_response[2] = ay / 16384.0;
                decoded_response[3] = az / 16384.0;
            }
            else if (sign == 2)
            {
                decoded_response[1] = ax / 16384.0;
                decoded_response[2] = -ay / 16384.0;
                decoded_response[3] = az / 16384.0;
            }
            else if (sign == 3)
            {
                decoded_response[1] = -ax / 16384.0;
                decoded_response[2] = -ay / 16384.0;
                decoded_response[3] = az / 16384.0;
            }
            else if (sign == 4)
            {
                decoded_response[1] = ax / 16384.0;
                decoded_response[2] = ay / 16384.0;
                decoded_response[3] = -az / 16384.0;
            }
            else if (sign == 5)
            {
                decoded_response[1] = -ax / 16384.0;
                decoded_response[2] = ay / 16384.0;
                decoded_response[3] = -az / 16384.0;
            }
            else if (sign == 6)
            {
                decoded_response[1] = ax / 16384.0;
                decoded_response[2] = -ay / 16384.0;
                decoded_response[3] = -az / 16384.0;
            }
            else if (sign == 7)
            {
                decoded_response[1] = -ax / 16384.0;
                decoded_response[2] = -ay / 16384.0;
                decoded_response[3] = -az / 16384.0;
            }
            else if (sign == 0)
            {
                decoded_response[1] = ax / 16384.0;
                decoded_response[2] = ay / 16384.0;
                decoded_response[3] = az / 16384.0;
            }
            decoded_response[4] = frame.can_dlc;
            decoded_response[5] = 1;
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void enable_motor(int motor_id)
{
    struct can_frame frame;
    frame.can_id = motor_id;
    frame.can_dlc = 8;
    frame.data[0] = 0xFF;
    frame.data[1] = 0xFF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFC;
    cout << "enabling ...." << endl;
    if (write(CAN_Socket, &frame, sizeof(struct can_frame)) == -1)
    {
        cout << "Write error1" << endl;
    }
    while (true)
    {
        struct can_frame received_frame;
        int bytes_read = read(CAN_Socket, &received_frame, sizeof(struct can_frame));
        if (bytes_read == -1)
        {
            cout << "Write error2" << endl;
        }
        if (bytes_read > 0)
        {
            if (received_frame.can_id == 0)
            { // if is motor CAN responce
                std::cout << "Motor  " << motor_id << "  is enabled" << endl;
                std::cout << "Motor CAN message ID: " << received_frame.can_id << endl;
                float motor_response[4];
                decodeMotorFrame(received_frame.data, motor_response);
                std::cout << "P: " << motor_response[1] << "     V: " << motor_response[2] << "     T: " << motor_response[3] << std::endl;
                break;
            }
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Define a function to read CAN data for a specific CAN ID in a separate thread
void ReadCANData(const std::string &interfaceName, uint32_t targetCANID, const std::string &outputFileName)
{
    float decodedValue[6];
    // Create a buffer for reading CAN frames
    struct can_frame frame;
    // Open the CSV file for writing
    std::ofstream outputFile(outputFileName, std::ios::out);
    if (!outputFile.is_open())
    {
        perror("File error");
        close(CAN_Socket);
        return;
    }
    outputFile << "Time , Ax3_thigh , Ay3_thigh , Az3_thigh , gx3_thigh , gy3_thigh , gz3_thigh , Ax4_shank , Ay4_shank , Az4_shank , gx4_shank , gy4_shank , gz4_shank , Ax5_foot , Ay5_foot , Az5_foot , gx5_foot , gy5_foot , gz5_foot , M1_position , M1_velocity , M1_torque , M2_position , M2_velocity , M2_torque , Hip_enc1 , Hip_enc2 , Back_ax , Back_ay , Back_az , Back_gx , Back_gy , Back_gz" << endl;

    while (true)
    {
        int Gyro_flag[3] = {0, 0, 0};
        int Acc_flag[3] = {0, 0, 0};
        int enc_flag[2] = {0, 0};
        int motor_flag[2] = {0, 0};
        int flags = 0;

        while (flags < 10)
        {

            int nbytes = read(CAN_Socket, &frame, sizeof(struct can_frame));
            if (nbytes < 0)
            {
                perror("Read error");
                break;
            }
            else
            {
                std::lock_guard<std::mutex> lock(std::mutex);
                // Decode the sensor data using a specific method
                decodeSensorData(frame, decodedValue);
                // Write decoded data to the CSV file
                if (decodedValue[0] == 1)
                {
                    mp1 = decodedValue[1];
                    mv1 = decodedValue[2];
                    mt1 = decodedValue[3];
                    motor_flag[0] = decodedValue[5];
                }
                else if (decodedValue[0] == 2)
                {
                    mp2 = decodedValue[1];
                    mv2 = decodedValue[2];
                    mt2 = decodedValue[3];
                    motor_flag[1] = decodedValue[5];
                }
                else if (decodedValue[0] == 5)
                {
                    if (decodedValue[4] == 7)
                    {
                        gx5 = decodedValue[1];
                        gy5 = decodedValue[2];
                        gz5 = decodedValue[3];
                        Gyro_flag[2] = decodedValue[5];
                    }
                    else if (decodedValue[4] == 8)
                    {
                        ax5 = decodedValue[1];
                        ay5 = decodedValue[2];
                        az5 = decodedValue[3];
                        Acc_flag[2] = decodedValue[5];
                    }
                }
                else if (decodedValue[0] == 3)
                {
                    if (decodedValue[4] == 7)
                    {
                        gx3 = decodedValue[1];
                        gy3 = decodedValue[2];
                        gz3 = decodedValue[3];
                        Gyro_flag[0] = decodedValue[5];
                    }
                    else if (decodedValue[4] == 8)
                    {
                        ax3 = decodedValue[1];
                        ay3 = decodedValue[2];
                        az3 = decodedValue[3];
                        Acc_flag[0] = decodedValue[5];
                    }
                }
                else if (decodedValue[0] == 4)
                {
                    if (decodedValue[4] == 7)
                    {
                        gx4 = decodedValue[1];
                        gy4 = decodedValue[2];
                        gz4 = decodedValue[3];
                        Gyro_flag[1] = decodedValue[5];
                    }
                    else if (decodedValue[4] == 8)
                    {
                        ax4 = decodedValue[1];
                        ay4 = decodedValue[2];
                        az4 = decodedValue[3];
                        Acc_flag[1] = decodedValue[5];
                    }
                }
                else if (decodedValue[0] == 1092)
                {
                    enc1 = decodedValue[1];
                    enc_flag[0] = decodedValue[5];
                }
                else if (decodedValue[0] == 1093)
                {
                    enc2 = decodedValue[1];
                    enc_flag[1] = decodedValue[5];
                }
            }
            flags = Gyro_flag[0] + Gyro_flag[1] + Gyro_flag[2] + Acc_flag[0] + Acc_flag[1] + Acc_flag[2] + enc_flag[0] + enc_flag[1] + motor_flag[0] + motor_flag[1];
            //
        }
        cout << "all sensor Gathered :))" << endl;
        gettimeofday(&time_now, nullptr);
        time_n = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);
        outputFile << time_n << "," << ax3 << "," << ay3 << "," << az3 << "," << gx3 << "," << gy3 << "," << gz3 << "," << ax4 << "," << ay4 << "," << az4 << "," << gx4 << "," << gy4 << "," << gz4 << "," << ax5 << "," << ay5 << "," << az5 << "," << gx5 << "," << gy5 << "," << gz5 << "," << mp1 << "," << mv1 << "," << mt1 << "," << mp2 << "," << mv2 << "," << mt2 << "," << enc1 << "," << enc2 << "," << back_ax << "," << back_ay << "," << back_az << "," << back_gx << "," << back_gy << "," << back_gz << std::endl;
    }
    outputFile.close(); // Close the CSV file and the socket when done
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Readi2cData(void)
{
    const char *device = "/dev/i2c-1"; // I2C device file
    int file;

    // Open the I2C device file
    if ((file = open(device, O_RDWR)) < 0)
    {
        perror("Failed to open the I2C device");
    }
    // Set the I2C slave address for MPU9250
    int addr = 0x68;
    if (ioctl(file, I2C_SLAVE, addr) < 0)
    {
        perror("Failed to set I2C address");
    }

    while (true)
    {
        char buffer[14]; // 14 bytes for accelerometer and gyroscope data

        if (write(file, &ACCEL_XOUT_H, 1) != 1)
        {
            perror("Failed to select accelerometer data register");
        }
        if (read(file, buffer, sizeof(buffer)) != sizeof(buffer))
        {
            perror("Failed to read from the I2C device");
        }
        // Process and print the received data
        int acx = (buffer[0] << 8) | buffer[1];
        if (acx & (1 << 16 - 1))
        {
            acx -= (1 << 16);
        }
        int acy = (buffer[2] << 8) | buffer[3];
        if (acy & (1 << 16 - 1))
        {
            acy -= (1 << 16);
        }
        int acz = (buffer[4] << 8) | buffer[5];
        if (acz & (1 << 16 - 1))
        {
            acz -= (1 << 16);
        }

        if (write(file, &gyro_XOUT_H, 1) != 1)
        {
            perror("Failed to select accelerometer data register");
        }
        if (read(file, buffer, sizeof(buffer)) != sizeof(buffer))
        {
            perror("Failed to read from the I2C device");
        }

        int gyx = (buffer[0] << 8) | buffer[1];
        if (gyx & (1 << 16 - 1))
        {
            gyx -= (1 << 16);
        }
        int gyy = (buffer[2] << 8) | buffer[3];
        if (gyy & (1 << 16 - 1))
        {
            gyy -= (1 << 16);
        }
        int gyz = (buffer[4] << 8) | buffer[5];
        if (gyz & (1 << 16 - 1))
        {
            gyz -= (1 << 16);
        }

        back_ax = acx / 16384.0;
        back_ay = acy / 16384.0;
        back_az = acz / 16384.0;
        back_gx = gyx / 131.0;
        back_gy = gyy / 131.0;
        back_gz = gyz / 131.0;
    }
    close(file);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SendToMotor(int motor_id)
{
    struct can_frame frame;
    frame.can_id = motor_id;
    frame.can_dlc = 8;
    frame.data[0] = 0x80;
    frame.data[1] = 0x00;
    frame.data[2] = 0x80;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x08;
    frame.data[7] = 0x00;

    while (true)
    {
        frame.can_id = 1;
        write(CAN_Socket, &frame, sizeof(struct can_frame));
        std::lock_guard<std::mutex> lock(std::mutex);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        frame.can_id = 2;
        write(CAN_Socket, &frame, sizeof(struct can_frame));
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
int main()
{
    gettimeofday(&time_now, nullptr);
    time_start = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);

    std::string canInterface = "can0"; // Adjust the interface name as needed

    CAN_Socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (CAN_Socket == -1)
    {
        perror("Socket creation error");
    }
    // Specify the CAN interface name (e.g., can0)
    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, "can0");
    ioctl(CAN_Socket, SIOCGIFINDEX, &ifr);

    // Bind the socket to the CAN interface
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(CAN_Socket, (struct sockaddr *)&addr, sizeof(addr)) == -1)
    {
        perror("Bind error");
        close(CAN_Socket);
    }

    /////////////////////////////////////////////////////////////////////////////////
    enable_motor(motor_ids[1]);
    std::cout << "moto1 enabled" << endl;
    enable_motor(motor_ids[0]);
    std::cout << "moto2 enabled" << endl;
    /////////////////////////////////////////////////////////////////////////////////////
    string filename;
    cout << "please input data logging number: ";
    cin >> filename;
    std::string outputFileName0 = "Data23_08/sensor_data" + filename + ".csv";
    // std::string outputFileName1 = "Data23_08/back_data"+filename+".csv";
    cout << endl
         << "Gathering Data from Sensors :) " << endl;

    // Define the target CAN IDs
    uint32_t targetCANID0 = 0;

    std::thread motorThread(SendToMotor, 1);
    std::thread canThread(ReadCANData, canInterface, targetCANID0, outputFileName0);
    std::thread i2cThread(Readi2cData);

    // Wait for the CAN threads to finish (optional)
    motorThread.join();
    canThread.join();
    i2cThread.join();

    return 0;
}
