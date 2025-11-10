#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include "esp_mac.h"
#include "Stellarsat-frames.h"
#include "Stellarsat-measure.h"

extern uint8_t modulationScheme;
extern uint8_t codeScheme;
extern uint8_t interleaver;
extern uint8_t BS_ID[2];
extern uint8_t SC_ID[2];

int create_header(uint8_t *hdr, uint8_t *size_hdr){//tamaño 33 byteshe
    uint8_t size[3]={size_hdr[0],size_hdr[1],size_hdr[2]};
    uint8_t type=0x1;   //Telemetry, not telecommand- Medio byte
    uint8_t modu=modulationScheme;
    uint8_t codi= codeScheme;
    uint8_t intleav= interleaver;   //Medio byte
    uint8_t ID[4]={BS_ID[0] ,BS_ID[1] ,SC_ID[0],SC_ID[1]};
    uint8_t reserved[2]={0,0};
    uint8_t header_type=1;  //Medio byte
    uint8_t P_version=1;    //Medio byte
    uint8_t crc[2]={0,0};//CRC calculation
    uint8_t header[15] = {size[0],size[1], size[2],((type & 0x0F) << 4) | (intleav & 0x0F),modu,
                    codi,ID[0],ID[1],ID[2],ID[3],((header_type & 0x0F) << 4) | (P_version & 0x0F),
                    reserved[0],reserved[1], crc[0], crc[1]};
    memcpy(hdr, header,sizeof(header));
    return 1;
}

int create_housekeeping(uint8_t *beacon_frame){
    beacon_frame[0]=SC_ID[0];
    beacon_frame[1]=SC_ID[1];
    beacon_frame[2]=1;
    beacon_frame[3]=(obc_app_status>>8)&0xFF;beacon_frame[4]=obc_app_status&0xFF;
    beacon_frame[5]=(obc_sys_check>>8)&0xFF;beacon_frame[6]=obc_sys_check&0xFF;
    beacon_frame[7]=(obc_timestamp_d>>8)&0xFF;beacon_frame[8]=obc_timestamp_d&0xFF;
    beacon_frame[9]=(obc_timestamp_ms>>24)&0xFF;beacon_frame[10]=(obc_timestamp_ms>>16)&0xFF;beacon_frame[11]=(obc_timestamp_ms>>8)&0xFF;beacon_frame[12]=obc_timestamp_ms&0xFF;
    beacon_frame[13]=(obc_time_reboot>>24)&0xFF;beacon_frame[14]=(obc_time_reboot>>16)&0xFF;beacon_frame[15]=(obc_time_reboot>>8)&0xFF;beacon_frame[16]=obc_time_reboot&0xFF;
    beacon_frame[17]=(obc_bootcounter>>24)&0xFF;beacon_frame[18]=(obc_bootcounter>>16)&0xFF;beacon_frame[19]=(obc_bootcounter>>8)&0xFF;beacon_frame[20]=obc_bootcounter&0xFF;
    beacon_frame[21]=(obc_reboot_cause>>8)&0xFF;beacon_frame[22]=obc_reboot_cause&0xFF;
    beacon_frame[23]=obc_sc_mode;
    beacon_frame[24]=obc_last_mode;
    beacon_frame[25]=(obc_mode_time>>24)&0xFF;beacon_frame[26]=(obc_mode_time>>16)&0xFF;beacon_frame[27]=(obc_mode_time>>8)&0xFF;beacon_frame[28]=obc_mode_time&0xFF;
    beacon_frame[29]=obc_mode;
    beacon_frame[30]=obc_last_ob_tle[0];beacon_frame[31]=obc_last_ob_tle[1];beacon_frame[32]=obc_last_ob_tle[2];beacon_frame[33]=obc_last_ob_tle[3];beacon_frame[34]=obc_last_ob_tle[4];
                    beacon_frame[35]=obc_last_ob_tle[5];beacon_frame[36]=obc_last_ob_tle[6];beacon_frame[37]=obc_last_ob_tle[7];beacon_frame[38]=obc_last_ob_tle[8];beacon_frame[39]=obc_last_ob_tle[9];
    beacon_frame[40]=obc_gps_state;
    beacon_frame[41]=(obc_gps_time>>16)&0xFF;beacon_frame[42]=(obc_gps_time>>8)&0xFF;beacon_frame[43]=obc_gps_time&0xFF;
    beacon_frame[44]=com_tc_list[0];beacon_frame[45]=com_tc_list[1];beacon_frame[46]=com_tc_list[2];beacon_frame[47]=com_tc_list[3];
                    beacon_frame[48]=com_tc_list[4];beacon_frame[49]=com_tc_list[5];beacon_frame[50]=com_tc_list[6];beacon_frame[51]=com_tc_list[7];
                    beacon_frame[52]=com_tc_list[8];beacon_frame[53]=com_tc_list[9];beacon_frame[54]=com_tc_list[10];beacon_frame[55]=com_tc_list[11];
                    beacon_frame[56]=com_tc_list[12];beacon_frame[57]=com_tc_list[13];beacon_frame[58]=com_tc_list[14];beacon_frame[59]=com_tc_list[15];
                    beacon_frame[60]=com_tc_list[16];beacon_frame[61]=com_tc_list[17];beacon_frame[62]=com_tc_list[18];beacon_frame[63]=com_tc_list[19];
                    beacon_frame[64]=com_tc_list[20];beacon_frame[65]=com_tc_list[21];beacon_frame[66]=com_tc_list[22];beacon_frame[67]=com_tc_list[23];
                    beacon_frame[68]=com_tc_list[24];beacon_frame[69]=com_tc_list[25];beacon_frame[70]=com_tc_list[26];beacon_frame[71]=com_tc_list[27];
                    beacon_frame[72]=com_tc_list[28];beacon_frame[73]=com_tc_list[29];beacon_frame[74]=com_tc_list[30];beacon_frame[75]=com_tc_list[31];
                    beacon_frame[76]=com_tc_list[32];beacon_frame[77]=com_tc_list[33];beacon_frame[78]=com_tc_list[34];beacon_frame[79]=com_tc_list[35];
                    beacon_frame[80]=com_tc_list[36];beacon_frame[81]=com_tc_list[37];beacon_frame[82]=com_tc_list[38];beacon_frame[83]=com_tc_list[39];
                    beacon_frame[84]=com_tc_list[40];beacon_frame[85]=com_tc_list[41];beacon_frame[86]=com_tc_list[42];beacon_frame[87]=com_tc_list[43];
                    beacon_frame[88]=com_tc_list[44];beacon_frame[89]=com_tc_list[45];beacon_frame[90]=com_tc_list[46];beacon_frame[91]=com_tc_list[47];
                    beacon_frame[92]=com_tc_list[48];beacon_frame[93]=com_tc_list[49];
    beacon_frame[94]=(com_tc_count>>24)&0xFF;beacon_frame[95]=(com_tc_count>>16)&0xFF;beacon_frame[96]=(com_tc_count>>8)&0xFF;beacon_frame[97]=com_tc_count&0xFF;
    beacon_frame[98]=(com_tm_count>>24)&0xFF;beacon_frame[99]=(com_tm_count>>16)&0xFF;beacon_frame[100]=(com_tm_count>>8)&0xFF;beacon_frame[101]=com_tm_count&0xFF;
    beacon_frame[102]=(com_rssi>>8)&0xFF;beacon_frame[103]=com_rssi&0xFF;
    beacon_frame[104]=(com_last_err>>8)&0xFF;beacon_frame[105]=com_last_err&0xFF;
    beacon_frame[106]=eps_mode;
    beacon_frame[107]=eps_bat_soc;
    beacon_frame[108]=(eps_sp_currxp>>8)&0xFF;beacon_frame[109]=eps_sp_currxp&0xFF;
    beacon_frame[110]=(eps_sp_currxm>>8)&0xFF;beacon_frame[111]=eps_sp_currxm&0xFF;
    beacon_frame[112]=(eps_sp_curryp>>8)&0xFF;beacon_frame[113]=eps_sp_curryp&0xFF;
    beacon_frame[114]=(eps_sp_currym>>8)&0xFF;beacon_frame[115]=eps_sp_currym&0xFF;
    beacon_frame[116]=(eps_sp_currzp>>8)&0xFF;beacon_frame[117]=eps_sp_currzp&0xFF;
    beacon_frame[118]=(eps_sp_voltxp>>8)&0xFF;beacon_frame[119]=eps_sp_voltxp&0xFF;
    beacon_frame[120]=(eps_sp_voltxm>>8)&0xFF;beacon_frame[121]=eps_sp_voltxm&0xFF;
    beacon_frame[122]=(eps_sp_voltyp>>8)&0xFF;beacon_frame[123]=eps_sp_voltyp&0xFF;
    beacon_frame[124]=(eps_sp_voltym>>8)&0xFF;beacon_frame[125]=eps_sp_voltym&0xFF;
    beacon_frame[126]=(eps_sp_voltzp>>8)&0xFF;beacon_frame[127]=eps_sp_voltzp&0xFF;
    beacon_frame[128]=(eps_pcdu_curr>>8)&0xFF;beacon_frame[129]=eps_pcdu_curr&0xFF;
    beacon_frame[130]=(eps_pcdu_volt>>8)&0xFF;beacon_frame[131]=eps_pcdu_volt&0xFF;
    beacon_frame[132]=adcs_mode;
    beacon_frame[133]=(adcs_imu_omega_x>>24)&0xFF;beacon_frame[134]=(adcs_imu_omega_x>>16)&0xFF;beacon_frame[135]=(adcs_imu_omega_x>>8)&0xFF;beacon_frame[136]=adcs_imu_omega_x&0xFF;
    beacon_frame[137]=(adcs_imu_omega_y>>24)&0xFF;beacon_frame[138]=(adcs_imu_omega_y>>16)&0xFF;beacon_frame[139]=(adcs_imu_omega_y>>8)&0xFF;beacon_frame[140]=adcs_imu_omega_y&0xFF;
    beacon_frame[141]=(adcs_imu_omega_z>>24)&0xFF;beacon_frame[142]=(adcs_imu_omega_z>>16)&0xFF;beacon_frame[143]=(adcs_imu_omega_z>>8)&0xFF;beacon_frame[144]=adcs_imu_omega_z&0xFF;
    beacon_frame[145]=(adcs_imu_mag_x>>24)&0xFF;beacon_frame[146]=(adcs_imu_mag_x>>16)&0xFF;beacon_frame[147]=(adcs_imu_mag_x>>8)&0xFF;beacon_frame[148]=adcs_imu_mag_x&0xFF;
    beacon_frame[149]=(adcs_imu_mag_y>>24)&0xFF;beacon_frame[150]=(adcs_imu_mag_y>>16)&0xFF;beacon_frame[151]=(adcs_imu_mag_y>>8)&0xFF;beacon_frame[152]=adcs_imu_mag_y&0xFF;
    beacon_frame[153]=(adcs_imu_mag_z>>24)&0xFF;beacon_frame[154]=(adcs_imu_mag_z>>16)&0xFF;beacon_frame[155]=(adcs_imu_mag_z>>8)&0xFF;beacon_frame[156]=adcs_imu_mag_z&0xFF;
    beacon_frame[157]=(adcs_q_x>>24)&0xFF;beacon_frame[158]=(adcs_q_x>>16)&0xFF;beacon_frame[159]=(adcs_q_x>>8)&0xFF;beacon_frame[160]=adcs_q_x&0xFF;
    beacon_frame[161]=(adcs_q_y>>24)&0xFF;beacon_frame[162]=(adcs_q_y>>16)&0xFF;beacon_frame[163]=(adcs_q_y>>8)&0xFF;beacon_frame[164]=adcs_q_y&0xFF;
    beacon_frame[165]=(adcs_q_z>>24)&0xFF;beacon_frame[166]=(adcs_q_z>>16)&0xFF;beacon_frame[167]=(adcs_q_z>>8)&0xFF;beacon_frame[168]=adcs_q_z&0xFF;
    beacon_frame[169]=(adcs_q_w>>24)&0xFF;beacon_frame[170]=(adcs_q_w>>16)&0xFF;beacon_frame[171]=(adcs_q_w>>8)&0xFF;beacon_frame[172]=adcs_q_w&0xFF;
    beacon_frame[173]=adcs_x_ecef;
    beacon_frame[174]=adcs_y_ecef;
    beacon_frame[175]=adcs_z_ecef;
    beacon_frame[176]=(the_sp_tempxp>>8)&0xFF;beacon_frame[177]=the_sp_tempxp&0xFF;
    beacon_frame[178]=(the_sp_tempxm>>8)&0xFF;beacon_frame[179]=the_sp_tempxm&0xFF;
    beacon_frame[180]=(the_sp_tempyp>>8)&0xFF;beacon_frame[181]=the_sp_tempyp&0xFF;
    beacon_frame[182]=(the_sp_tempym>>8)&0xFF;beacon_frame[183]=the_sp_tempym&0xFF;
    beacon_frame[184]=(the_sp_tempzp>>8)&0xFF;beacon_frame[185]=the_sp_tempzp&0xFF;
    beacon_frame[186]=(the_adcs_temp>>8)&0xFF;beacon_frame[187]=the_adcs_temp&0xFF;
    beacon_frame[188]=(the_obc_temp>>8)&0xFF;beacon_frame[189]=the_obc_temp&0xFF;
    beacon_frame[190]=(the_rfboard_temp>>8)&0xFF;beacon_frame[191]=the_rfboard_temp&0xFF;
    beacon_frame[192]=(the_pcdu_temp>>8)&0xFF;beacon_frame[193]=the_pcdu_temp&0xFF;
    beacon_frame[194]=the_heater_state;
    beacon_frame[195]=(the_heater_time>>8)&0xFF;beacon_frame[196]=the_heater_time&0xFF;
    beacon_frame[197]=pay_num;
    beacon_frame[198]=(pay_time_last>>8)&0xFF;beacon_frame[199]=pay_time_last&0xFF;

    return 1;
}

int create_telemetry(uint8_t *telemetry_frame, int byte_start, int byte_end, uint16_t meas_id){
    telemetry_frame[0]=SC_ID[0];       
    telemetry_frame[1]=SC_ID[1];
    telemetry_frame[2]=tm_type;
    telemetry_frame[3]=(obc_timestamp_d>>8);telemetry_frame[4]=obc_timestamp_d&0xFF;
    telemetry_frame[5]=(obc_timestamp_ms>>24);telemetry_frame[6]=(obc_timestamp_ms>>16);telemetry_frame[7]=(obc_timestamp_ms>>8);telemetry_frame[8]=obc_timestamp_ms&0xFF;
    telemetry_frame[9]=(adcs_imu_omega_x>>24);telemetry_frame[10]=(adcs_imu_omega_x>>16);telemetry_frame[11]=(adcs_imu_omega_x>>8);telemetry_frame[12]=adcs_imu_omega_x&0xFF;
    telemetry_frame[13]=(adcs_imu_omega_y>>24);telemetry_frame[14]=(adcs_imu_omega_y>>16);telemetry_frame[15]=(adcs_imu_omega_y>>8);telemetry_frame[16]=adcs_imu_omega_y&0xFF;
    telemetry_frame[17]=(adcs_imu_omega_z>>24);telemetry_frame[18]=(adcs_imu_omega_z>>16);telemetry_frame[19]=(adcs_imu_omega_z>>8);telemetry_frame[20]=adcs_imu_omega_z&0xFF;
    telemetry_frame[21]=(adcs_imu_mag_x>>24);telemetry_frame[22]=(adcs_imu_mag_x>>16);telemetry_frame[23]=(adcs_imu_mag_x>>8);telemetry_frame[24]=adcs_imu_mag_x&0xFF;
    telemetry_frame[25]=(adcs_imu_mag_y>>24);telemetry_frame[26]=(adcs_imu_mag_y>>16);telemetry_frame[27]=(adcs_imu_mag_y>>8);telemetry_frame[28]=adcs_imu_mag_y&0xFF;
    telemetry_frame[29]=(adcs_imu_mag_z>>24);telemetry_frame[30]=(adcs_imu_mag_z>>16);telemetry_frame[31]=(adcs_imu_mag_z>>8);telemetry_frame[32]=adcs_imu_mag_z&0xFF;
    telemetry_frame[33]=(adcs_q_x>>24);telemetry_frame[34]=(adcs_q_x>>16);telemetry_frame[35]=(adcs_q_x>>8);telemetry_frame[36]=adcs_q_x&0xFF;
    telemetry_frame[37]=(adcs_q_y>>24);telemetry_frame[38]=(adcs_q_y>>16);telemetry_frame[39]=(adcs_q_y>>8);telemetry_frame[40]=adcs_q_y&0xFF;
    telemetry_frame[41]=(adcs_q_z>>24);telemetry_frame[42]=(adcs_q_z>>16);telemetry_frame[43]=(adcs_q_z>>8);telemetry_frame[44]=adcs_q_z&0xFF;
    telemetry_frame[45]=(adcs_q_w>>24);telemetry_frame[46]=(adcs_q_w>>16);telemetry_frame[47]=(adcs_q_w>>8);telemetry_frame[48]=adcs_q_w&0xFF;
    telemetry_frame[49]=adcs_x_ecef;
    telemetry_frame[50]=adcs_y_ecef;
    telemetry_frame[51]=adcs_z_ecef;
    telemetry_frame[52]=(meas_id>>8);telemetry_frame[53]=meas_id&0xFF;
    telemetry_frame[54]=pay_nb_tf;
    telemetry_frame[55]=(pay_meas_time>>8);telemetry_frame[56]=pay_meas_time&0xFF;
    memcpy(&telemetry_frame[57], &stellar_measure[byte_start],(byte_end-byte_start));
    return 1;
}