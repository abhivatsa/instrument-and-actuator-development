#ifndef SHARED_OBJECT_H
#define SHARED_OBJECT_H

//#define DRIVE_CNT 6

struct DriveRxPDO
{
    uint16_t controlword[6];
    int8_t   modes_of_operation[6];
    int16_t  target_torque[6];
    int32_t  target_position[6];
    int32_t  target_velocity[6];
    int16_t  torque_offset[6];
    int32_t  velocity_offset[6];
    uint32_t digital_physical_output[6];
    uint32_t digital_output_bit_mask[6];
};

struct DriveTxPDO
{

    bool isAllDrivesSwitchedOn = false;
    uint16_t statusword[6];
    int8_t   mode_of_operation_display[6];
    int32_t  position_actual_value[6];
    int32_t  velocity_actual_value[6];
    int16_t  torque_actual_value[6];
    uint32_t digital_input_value[6];
    uint16_t error_code[6];
};


#endif
