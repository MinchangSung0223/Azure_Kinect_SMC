'''
Created on 2019. 6. 17.

@author: YJHeo
'''
debugging = False


import socket
import sys
import numpy as np

from ctypes import *
from threading import Lock
###############################################################################
# Robot Interface                                                             #
###############################################################################
SIZE_HEADER = 52
SIZE_COMMAND = 4
SIZE_HEADER_COMMAND = 56
SIZE_DATA_TCP_MAX  = 200
SIZE_DATA_MAX = 200
SIZE_DATA_ASCII_MAX = 32
SIZE_PACKET = 256


###############################################################################
# Robot Type                                                                  #
###############################################################################
ROBOT_NAME_DEFAULT    = "NRMK-Indy7"

ROBOT_INDYRP    = "NRMK-IndyRP"
ROBOT_INDYRP2   = "NRMK-IndyRP2"
ROBOT_INDY3     = "NRMK-Indy3"
ROBOT_INDY5     = "NRMK-Indy5"
ROBOT_INDY10    = "NRMK-Indy10"
ROBOT_INDY7     = "NRMK-Indy7"
ROBOT_INDY15    = "NRMK-Indy15"
ROBOT_OPTI5     = "NRMK-OPTi5"
ROBOT_OPTI10    = "NRMK-OPTi10"

###############################################################################
# C-type Data                                                                 #
###############################################################################
class HeaderCommandStruct(Structure):
    _pack_ = 1
    _fields_ = [("robotName", c_ubyte * 20),
                ("robotVersion", c_ubyte * 12),
                ("stepInfo", c_ubyte),
                ("sof", c_ubyte),
                ("invokeId", c_uint32),
                ("dataSize", c_uint32),
                ("status", c_ubyte * 4),
                ("reserved", c_ubyte * 6),
                ("cmdId", c_uint32)]


class HeaderCommand(Union):
    _fields_ = [("byte", c_ubyte * SIZE_DATA_TCP_MAX),
                ("val", HeaderCommandStruct)]


class Data(Union):
    _fields_ = [("byte", c_ubyte * SIZE_DATA_TCP_MAX),
                ("asciiStr", c_ubyte * (SIZE_DATA_ASCII_MAX + 1)),
                ("str", c_ubyte * 200),
                ("charVal", c_ubyte),
                ("boolVal", c_byte),
                ("shortVal", c_uint16),
                ("intVal", c_int32),
                ("floatVal", c_float),
                ("doubleVal", c_double),
                ("byteVal", c_ubyte),
                ("wordVal", c_ubyte * 2),
                ("uwordVal", c_ubyte * 2),
                ("dwordVal", c_ubyte * 4),
                ("lwordVal", c_ubyte * 8),
                ("bool6dArr", c_ubyte * 6),
                ("bool7dArr", c_ubyte * 7),
                ("boolArr", c_ubyte * 200),
                ("char2dArr", c_ubyte * 2),
                ("char3dArr", c_ubyte * 3),
                ("char6dArr", c_ubyte * 6),
                ("char7dArr", c_ubyte * 7),
                ("charArr", c_ubyte * 200),
                ("int2dArr", c_int32 * 2),
                ("int3dArr", c_int32 * 3),
                ("int6dArr", c_int32 * 6),
                ("int7dArr", c_int32 * 7),
                ("intArr", c_int32 * 50),
                ("float3dArr", c_float * 3),
                ("float6dArr", c_float * 6),
                ("float7dArr", c_float * 7),
                ("floatArr", c_float * 50),
                ("double3dArr", c_double * 3),
                ("double6dArr", c_double * 6),
                ("double7dArr", c_double * 7),
                ("doubleArr", c_double * 50),
                ("byteArr", c_ubyte * 200),
                ("wordArr", c_ubyte * 2*100),
                ("uwordArr", c_ubyte * 2*100),
                ("dwordArr", c_ubyte * 4*50),
                ("lwordArr", c_ubyte * 8*25)]

class Packet(Union):
    _fields_ = [("header", HeaderCommand),
                ("data", Data)]

class DIO(Structure):
    _fields_ = [("channel", c_uint32),
                ("value", c_ubyte)]


#########################################################################
# Command                                                               #
#########################################################################
CMD_CHECK                                   = 0
CMD_EMERGENCY_STOP                          = 1
CMD_RESET_ROBOT                             = 2
CMD_SET_SERVO                               = 3
CMD_SET_BRAKE                               = 4
CMD_STOP                                    = 5
CMD_MOVE                                    = 6
CMD_MOVE_HOME                               = 7
CMD_MOVE_ZERO                               = 8
CMD_JOINT_MOVE_TO                           = 9
CMD_JOINT_MOVE_BY                           = 10
CMD_TASK_MOVE_TO                            = 11
CMD_TASK_MOVE_BY                            = 12

CMD_START_CURRENT_PROGRAM                   = 14
CMD_PAUSE_CURRENT_PROGRAM                   = 15
CMD_RESUME_CURRENT_PROGRAM                  = 16
CMD_STOP_CURRENT_PROGRAM                    = 17
CMD_START_DEFAULT_PROGRAM                   = 18
CMD_REGISTER_DEFAULT_PROGRAM_IDX            = 19
CMD_GET_REGISTERED_DEFAULT_PROGRAM_IDX      = 20

CMD_IS_ROBOT_RUNNING                        = 30
CMD_IS_READY                                = 20
CMD_IS_EMG                                  = 32
CMD_IS_COLLIDED                             = 33
CMD_IS_ERR                                  = 24
CMD_IS_BUSY                                 = 21
CMD_IS_MOVE_FINISEHD                        = 36
CMD_IS_HOME                                 = 22
CMD_IS_ZERO                                 = 23
CMD_IS_IN_RESETTING                         = 39
CMD_IS_DIRECT_TECAHING                      = 60
CMD_IS_TEACHING                             = 61
CMD_IS_PROGRAM_RUNNING                      = 62
CMD_IS_PROGRAM_PAUSED                       = 63
CMD_IS_CONTY_CONNECTED                      = 64

CMD_CHANGE_DIRECT_TEACHING                  = 80
CMD_FINISH_DIRECT_TEACHING                  = 81

CMD_JOINT_PUSH_BACK_WAYPOINT_SET            = 90
CMD_JOINT_POP_BACK_WAYPOINT_SET             = 91
CMD_JOINT_CLEAR_WAYPOINT_SET                = 92
CMD_JOINT_EXECUTE_WAYPOINT_SET              = 94
CMD_TASK_PUSH_BACK_WAYPOINT_SET             = 95
CMD_TASK_POP_BACK_WAYPOINT_SET              = 96
CMD_TASK_CLEAR_WAYPOINT_SET                 = 97
CMD_TASK_EXECUTE_WAYPOINT_SET               = 99

CMD_SET_DEFAULT_TCP                         = 100
CMD_RESET_DEFAULT_TCP                       = 101
CMD_SET_COMP_TCP                            = 102
CMD_RESET_COMP_TCP                          = 103
CMD_SET_REFFRAME                            = 104
CMD_RESET_REFFRAME                          = 105
CMD_SET_COLLISION_LEVEL                     = 104
CMD_SET_JOINT_BOUNDARY                      = 105
CMD_SET_TASK_BOUNDARY                       = 106
CMD_SET_JOINT_BLEND_RADIUS_LEVEL            = 107  # DEPRECATED
CMD_SET_TASK_BLEND_RADIUS_LEVEL             = 108 # DEPRECATED
CMD_SET_JOINT_WTIME                         = 109
CMD_SET_TASK_WTIME                          = 110
CMD_SET_TASK_CMODE                          = 113
CMD_SET_JOINT_BLEND_RADIUS                  = 107
CMD_SET_TASK_BLEND_RADIUS                   = 108

CMD_GET_DEFAULT_TCP                         = 200
CMD_GET_COMP_TCP                            = 201
CMD_GET_REFFRAME                            = 202
CMD_GET_COLLISION_LEVEL                     = 203
CMD_GET_JOINT_BOUNDARY                      = 204
CMD_GET_TASK_BOUNDARY                       = 205
# CMD_GET_JOINT_BLEND_RADIUS                  = 206  # DEPRECATED
# CMD_GET_TASK_BLEND_RADIUS                   = 207  # DEPRECATED
CMD_GET_JOINT_WTIME                         = 208
CMD_GET_TASK_WTIME                          = 209
CMD_GET_TASK_CMODE                          = 210
CMD_GET_JOINT_BLEND_RADIUS                  = 213
CMD_GET_TASK_BLEND_RADIUS                   = 214

CMD_GET_RUNNING_TIME                        = 300
CMD_GET_CMODE                               = 301
CMD_GET_JOINT_STATE                         = 302
CMD_GET_JOINT_POSITION                      = 320
CMD_GET_JOINT_VELOCITY                      = 321
CMD_GET_TASK_POSITION                       = 322
CMD_GET_TASK_VELOCITY                       = 323
CMD_GET_TORQUE                              = 324

CMD_GET_LAST_EMG_INFO                       = 380

CMD_GET_SMART_DI                            = 400
CMD_GET_SMART_DIS                           = 401
CMD_SET_SMART_DO                            = 402
CMD_SET_SMART_DOS                           = 403
CMD_GET_SMART_AI                            = 404
CMD_SET_SMART_AO                            = 405
CMD_GET_SMART_DO                            = 406
CMD_GET_SMART_DOS                           = 407
CMD_GET_SMART_AO                            = 408

CMD_GET_EXTIO_FTCAN_ROBOT_RAW               = 420
CMD_GET_EXTIO_FTCAN_ROBOT_TRANS             = 421
CMD_GET_EXTIO_FTCAN_CB_RAW                  = 422
CMD_GET_EXTIO_FTCAN_CB_TRANS                = 423

CMD_READ_DIRECT_VARIABLE                    = 460
CMD_READ_DIRECT_VARIABLES                   = 461
CMD_WRITE_DIRECT_VARIABLE                   = 462
CMD_WRITE_DIRECT_VARIABLES                  = 463

# CMD_MAKE_PROG_INIT						    = 700
# CMD_MAKE_PROG_CLEAR						    = 701
# CMD_MAKE_PROG_SET						    = 702
# CMD_MAKE_PROG_ADD_MOVE_HOME				    = 710
# CMD_MAKE_PROG_ADD_MOVE_ZERO				    = 711
# CMD_MAKE_PROG_ADD_MOVE_JOINT_MOVE_TO	    = 712
# CMD_MAKE_PROG_ADD_MOVE_JOINT_MOVE_BY	    = 713
# CMD_MAKE_PROG_ADD_MOVE_TASK_MOVE_TO		    = 714
# CMD_MAKE_PROG_ADD_MOVE_TASK_MOVE_BY		    = 715
# CMD_MAKE_PROG_ADD_WAIT					    = 720
# CMD_MAKE_PROG_ADD_WAIT_FOR				    = 721
# CMD_MAKE_PROG_ADD_ASSIGN_SMART_DO		    = 730
# CMD_MAKE_PROG_ADD_ASSIGN_SMART_AO		    = 731
# CMD_MAKE_PROG_ADD_ASSIGN_ENDTOOL_AO		    = 731
# CMD_MAKE_PROG_ADD_TOOL_COMMAND			    = 740
# CMD_MAKE_PROG_REMOVE_LAST_COMMAMND		    = 770
CMD_INIT_CUSTOM_PROGRAM				        = 500
CMD_ADD_MOVEHOME_CUSTOM_PROGRAM 	        = 501
CMD_ADD_JOINTMOVETO_CUSTOM_PROGRAM 	        = 502
CMD_ADD_TASKMOVETO_CUSTOM_PROGRAM 	        = 503
CMD_ADD_WAIT_CUSTOM_PROGRAM 		        = 504
CMD_ADD_DOCONTROL_CUSTOM_PROGRAM 	        = 505
CMD_ADD_TOOLCOMMAND_CUSTOM_PROGRAM 	        = 506

CMD_SET_CUSTOM_PROGRAM				        = 510
CMD_CLEAR_CUSTOM_PROGRAM			        = 511
CMD_ADD_TASKMOVEBY_CUSTOM_PROGRAM 	        = 512

CMD_FOR_EXTENDED				            = 800
CMD_FOR_STREAMING				            = 801

CMD_SEND_KEYCOMMAND			                = 9996
CMD_READ_MEMORY				                = 9997
CMD_WRITE_MEMORY			                = 9998
CMD_ERROR					                = 9999

#########################################################################
# Extended DCP command                                                  #
#########################################################################
EXT_CMD_MOVE_TRAJ_BY_DATA		        = 1
EXT_CMD_MOVE_TRAJ_BY_TXT_DATA	        = 2
EXT_CMD_MOVE_TRAJ_BY_FILE		        = 3
EXT_CMD_MOVE_TRAJ_BY_TXT_FILE	        = 4

EXT_CMD_JMOVE_ABS_WAYPOINT_SET		    = 11
EXT_CMD_TMOVE_ABS_WAYPOINT_SET		    = 12

EXT_CMD_SET_JSON_PROG                   = 21
EXT_CMD_SET_JSON_PROG_START             = 22


#########################################################################
# Error code                                                            #
#########################################################################
ERR_NONE                 = 0
ERR_NO_MATCHED_ROBOT     = 1
ERR_NO_MATCHED_STEP      = 2
ERR_HEADER_FORMAT        = 4
ERR_OVER_DATA_SIZE       = 5
ERR_NOT_SUPPORT_COMMAND  = 6
ERR_UNKNOWN_COMMAND      = 7
ERR_UNKNOWN_DATA         = 8
ERR_PROCESS_FAILED       = 9
ERR_PARSE_FAILED         = 10
ERR_NO_MATCHED_PARAMETER = 11
ERR_NO_MATCHED_DATA_SIZE = 12
ERR_WRONG_ASCII_FORMAT   = 13
ERR_ROBOT_MOVING_STATE   = 14
ERR_ROBOT_PROGRAM_RUNNING = 15
ERR_ROBOT_MOVE_FAILED     = 16
ERR_NO_DEFAULT_PROGRAM    = 17
ERR_NO_CURRENT_PROGRAM    = 18
ERR_CURRENT_PROGRAM_STATE = 19
ERR_EMG_STATE             = 20
ERR_ROBOT_STATE           = 21
ERR_ROBOT_PROGRAM_LOAD_FAILED = 22
ERR_DIRECT_VARIABLE_INVALID_ADDRESS = 23
ERR_DIRECT_VARIABLE_INVALID_FORMAT = 24
ERR_DIRECT_VARIABLE_REFNUM_LIMIT = 25
ERR_CONNECTION_EXCEPTION = 600
ERR_CONNECTION_TIMEOUT = 601
def err_to_string(err_cmd):
    return  {ERR_NONE: "ErrorCode {}: No Error".format(err_cmd),
              ERR_NO_MATCHED_ROBOT: "ErrorCode {}: Not matched robot".format(err_cmd),
              ERR_NO_MATCHED_STEP: "ErrorCode {}: Not matched step".format(err_cmd),
              ERR_HEADER_FORMAT: "ErrorCode {}: Invalid header format".format(err_cmd),
              ERR_OVER_DATA_SIZE: "ErrorCode {}: Over data size".format(err_cmd),
              ERR_NOT_SUPPORT_COMMAND: "ErrorCode {}: Unsupported command".format(err_cmd),
              ERR_UNKNOWN_COMMAND: "ErrorCode {}: Unknown command".format(err_cmd),
              ERR_UNKNOWN_DATA: "ErrorCode {}: Unknown data".format(err_cmd),
              ERR_PROCESS_FAILED: "ErrorCode {}: Process fail".format(err_cmd),
              ERR_PARSE_FAILED: "ErrorCode {}: Parsing fail (data error)".format(err_cmd),
              ERR_NO_MATCHED_PARAMETER: "ErrorCode {}: Not matched data type".format(err_cmd),
              ERR_NO_MATCHED_DATA_SIZE: "ErrorCode {}: Not matched data size ".format(err_cmd),
              # ERR_WRONG_ASCII_FORMAT: "ErrorCode {}: ".format(err_cmd),
              ERR_ROBOT_MOVING_STATE: "ErrorCode {}: Robot is moving".format(err_cmd),
              ERR_ROBOT_PROGRAM_RUNNING: "ErrorCode {}: Robot program is running".format(err_cmd),
              ERR_ROBOT_MOVE_FAILED: "ErrorCode {}: Move fail".format(err_cmd),
              ERR_NO_DEFAULT_PROGRAM: "ErrorCode {}: No default program".format(err_cmd),
              ERR_NO_CURRENT_PROGRAM: "ErrorCode {}: No loaded program".format(err_cmd),
              ERR_CURRENT_PROGRAM_STATE: "ErrorCode {}: No proper program state".format(err_cmd),
              ERR_EMG_STATE: "ErrorCode {}: Robot is emergency state".format(err_cmd),
              ERR_ROBOT_STATE: "ErrorCode {}: Not proper robot state".format(err_cmd),
              ERR_ROBOT_PROGRAM_LOAD_FAILED: "ErrorCode {}: Program load fail".format(err_cmd),
              ERR_DIRECT_VARIABLE_INVALID_ADDRESS: "ErrorCode {}: Invalid direct variable address".format(err_cmd),
              ERR_DIRECT_VARIABLE_INVALID_FORMAT: "ErrorCode {}: Invalid direct variable format".format(err_cmd),
              ERR_DIRECT_VARIABLE_REFNUM_LIMIT: "ErrorCode {}: Limit of direct variable size".format(err_cmd) }.get(err_cmd, "None")



#########################################################################
# Header Status Bit                                                     #
#########################################################################
HEADER_STATUS_BIT_TASK_RUNNING		= 0x80000000	# 0b 1000 0000 0000 0000 0000 0000 0000 0000
HEADER_STATUS_BIT_ROBOT_READY		= 0x40000000	# 0b 0100 0000 0000 0000 0000 0000 0000 0000
HEADER_STATUS_BIT_EMG_STOPPED		= 0x20000000	# 0b 0010 0000 0000 0000 0000 0000 0000 0000
HEADER_STATUS_BIT_COLLIDED			= 0x10000000	# 0b 0001 0000 0000 0000 0000 0000 0000 0000
HEADER_STATUS_BIT_ERR_STATE			= 0x08000000	# 0b 0000 1000 0000 0000 0000 0000 0000 0000
HEADER_STATUS_BIT_BUSY				= 0x04000000	# 0b 0000 0100 0000 0000 0000 0000 0000 0000
HEADER_STATUS_BIT_MOVE_FINISHED		= 0x02000000	# 0b 0000 0010 0000 0000 0000 0000 0000 0000
HEADER_STATUS_BIT_HOME				= 0x01000000	# 0b 0000 0001 0000 0000 0000 0000 0000 0000
HEADER_STATUS_BIT_ZERO				= 0x00800000	# 0b 0000 0000 1000 0000 0000 0000 0000 0000
HEADER_STATUS_BIT_IN_RESETTING		= 0x00400000	# 0b 0000 0000 0100 0000 0000 0000 0000 0000

HEADER_STATUS_BIT_DIRECT_TEACHING	= 0x00000080	# 0b 0000 0000 0000 0000 0000 0000 1000 0000
HEADER_STATUS_BIT_TEACHING			= 0x00000040	# 0b 0000 0000 0000 0000 0000 0000 0100 0000
HEADER_STATUS_BIT_PROGRAM_RUNNING	= 0x00000020	# 0b 0000 0000 0000 0000 0000 0000 0010 0000
HEADER_STATUS_BIT_PROGRAM_PAUSED	= 0x00000010	# 0b 0000 0000 0000 0000 0000 0000 0001 0000
HEADER_STATUS_BIT_CONTY_CONNECTED	= 0x00000008	# 0b 0000 0000 0000 0000 0000 0000 0000 1000
#########################################################################
# DirectVariableType                                                    #
#########################################################################
DIRECT_VAR_TYPE_ERROR       = -1
DIRECT_VAR_TYPE_BYTE        = 0
DIRECT_VAR_TYPE_WORD        = 1
DIRECT_VAR_TYPE_DWORD       = 2
DIRECT_VAR_TYPE_LWORD       = 3
DIRECT_VAR_TYPE_FLOAT       = 4
DIRECT_VAR_TYPE_DFLOAT      = 5
DIRECT_VAR_TYPE_MODBUS_REG  = 10

###############################################################################
# Debug                                                                       #
###############################################################################
def dump_buf(msg, buf, length) :
    if debugging:
        print(msg)
        for i in range (0, length):
            print(i, end=' - ')
            print(buf[i])

###############################################################################
# Indy Client Class                                                           #
###############################################################################
class IndyDCPClient:
    def __init__(self, bind_ip, server_ip, robot_name, robot_version=""):
        self.__server_port  = 6066
        self.__sof_server = 0x12
        self.__sof_client = 0x34
        self.__step_ver = 0x02
        self.__lock = Lock()
        self.__is_connected = False

        self.time_out = 10
        self.v_invokeId = 0

        self.bind_ip = bind_ip
        self.server_ip = server_ip
        self.robot_name = robot_name
        self.robot_version = robot_version

        self.joint_dof = 7 if self.robot_name==ROBOT_INDYRP2 else 6
        self.sock_fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self):
        self.__lock.acquire()
        self.sock_fd.bind((self.bind_ip, 0))
        try:
            self.sock_fd.connect((self.server_ip, self.__server_port))
        except socket.error as e:
            print("Socket connection error: {}".format(e))
            self.sock_fd.close()
            self.__lock.release()
            return False
        else:
            self.__lock.release()
            print("Connect: Bind IP ({bin_ip}), Server IP ({ser_ip})".format(bin_ip=self.bind_ip, ser_ip=self.server_ip))
            self.__is_connected = True
            return True

    def disconnect(self):
        self.__lock.acquire()
        self.sock_fd.close()
        self.__lock.release()
        self.__is_connected = False
        print("Disconnected")

    def shutdown(self):
        self.sock_fd.shutdown(socket.SHUT_RDWR)
        self.__is_connected = False
        print("Shut down")


    def is_connected(self):
        if not self.__is_connected:
            return False

        self.__lock.acquire()
        try:
            ret_val = self.sock_fd.getsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE)
        except socket.error as e:
            print("{}".format(e))
            self.sock_fd.close()
            self.__lock.release()
            return False
        else:
            if ret_val != 0:
                print("Invalid Socket")
                return False
            self.__lock.release()
            return True

    def set_timeout_sec(self, time_out):
        if time_out < 0:
            print("Invalid time out setting: {}<0".format(time_out))
        self.time_out = time_out

    def _send_message(self, buf, size):
        dump_buf("SendBytes: ", buf, size)
        total_sent = 0
        while total_sent < size:
            self.sock_fd.settimeout(self.time_out)
            sent = self.sock_fd.send(buf[total_sent:size])
            if sent == -1:
                print('Error: sent == -1')
                return -1
            elif sent == 0:
                self.__lock.release()
                print('Error: sent == 0')
                return -1
            total_sent = total_sent + sent
        return 0

    def _recv_message(self, buf, size):
        chunks = []
        bytes_recd = 0
        while bytes_recd < size:
            self.sock_fd.settimeout(self.time_out)
            chunk = self.sock_fd.recv(size - bytes_recd)
            if chunk == b'':
                print('Error: receive error')
                memset (buf, 0, sizeof (buf))
                self.__lock.release()
                self.shutdown()
                return -1
            chunks.append(chunk)
            if (bytes_recd + len(chunk)) > sizeof (buf):
                break
            bytes_recd += len(chunk)
        data = b''.join(chunks)
        memset(buf, 0, sizeof (buf))
        memmove(buf, data, len(data))
        return buf

    def check_header(self, req=HeaderCommand(), res=HeaderCommand(), err_code=ERR_NONE):
        req_robot_name = np.array(req.val.robotName).tostring().decode('utf-8')
        res_robot_name = np.array(res.val.robotName).tostring().decode('utf-8')
        if req_robot_name != res_robot_name:
            print("Header check fail (robotName): Request {_req}, Response {_res}".format(_req=req_robot_name, _res=res_robot_name))
        if req.val.stepInfo != res.val.stepInfo:
            print("Header check fail (stepInfo): Request {_req}, Response {_res}".format(_req=req.val.stepInfo, _res=res.val.stepInfo))
        if req.val.invokeId != res.val.invokeId:
            print("Header check fail (invokeId): Request {_req}, Response {_res}".format(_req=req.val.invokeId, _res=res.val.invokeId))
        if res.val.sof != self.__sof_server:
            print("Header check fail (sof): Request {_req}, Response {_res}".format(_req=self.__sof_server, _res=res.val.sof))
        if req.val.cmdId != res.val.cmdId:
            print("Header check fail (cmdId): Request {_req}, Response {_res}".format(_req=req.val.cmdId, _res=res.val.cmdId))
        if res.val.cmdId==CMD_ERROR:
            print(err_to_string(err_code))
            return err_code
        return ERR_NONE

    def _handle_command(self, cmd, req_data=Data(), req_data_size=0):
        #print("HANDLE COMMAND")
        #print("CMD : "+str(cmd)+"REQ_DATA : "+str(Data()))
        self.__lock.acquire()

        write_buffer = (c_char* 1024)()
        read_buffer = (c_char* 1024)()

        if req_data_size > SIZE_DATA_TCP_MAX or req_data_size < 0:
            self.__lock.release()
            self.disconnect()
            raise Exception("Request size is invalid {}: Disconnected".format(req_data_size))

        # Make header
        req_header = HeaderCommand()
        memset(req_header.byte, 0, sizeof(req_header.byte))

        b_str_robot_name = self.robot_name.encode('ascii')
        memmove(req_header.val.robotName, c_char_p(b_str_robot_name), len(self.robot_name))

        b_str_robot_ver = self.robot_version.encode('ascii')
        memmove(req_header.val.robotVersion, c_char_p(b_str_robot_ver), len(self.robot_version))

        req_header.val.stepInfo = self.__step_ver
        req_header.val.sof = self.__sof_client

        req_header.val.cmdId = cmd
        req_header.val.dataSize = req_data_size

        self.v_invokeId += 1
        req_header.val.invokeId = self.v_invokeId

        # Send packet to socket
        memmove(write_buffer, req_header.byte, SIZE_HEADER_COMMAND)
        self._send_message(write_buffer, SIZE_HEADER_COMMAND)
        if req_data_size > 0:
            if hasattr(req_data, 'byte'):
                memmove(write_buffer, req_data.byte, req_data_size)
            else:
                memmove(write_buffer, req_data, req_data_size) # For execute command move
            self._send_message(write_buffer, req_data_size)

        # Recv header from socket
        res_header = HeaderCommand()
        read_buffer = self._recv_message(read_buffer, SIZE_HEADER_COMMAND)
        memmove(res_header.byte, read_buffer, SIZE_HEADER_COMMAND)

        # Recv data from socket
        res_data = Data()
        res_data_size = res_header.val.dataSize
        if res_data_size > SIZE_DATA_TCP_MAX or res_data_size < 0:
            print("Response data size is invalid {} (max: {}): Disconnected".format(res_data_size, SIZE_DATA_TCP_MAX))
            self.__lock.release()
            self.disconnect()
        elif res_data_size > 0:
            read_buffer = self._recv_message(read_buffer, res_data_size)
            memmove(res_data.byte, read_buffer, res_data_size)

        self.__lock.release()

        # Check header and error
        error_code = self.check_header(req_header, res_header, res_data.intVal)
        return error_code, res_data, res_data_size

    def _handle_extended_command(self, ext_cmd, req_ext_data, req_ext_data_size=0):
        self.__lock.acquire()
        ret = False

        write_buffer = (c_char * 1024)()
        read_buffer = (c_char * 1024)()

        if req_ext_data_size > sys.maxsize or req_ext_data_size < 0:
            self.__lock.release()
            self.disconnect()
            print("Send data size error")
        if req_ext_data_size > 0 and req_ext_data is None:
            print("Send data error: Null data")

        # Make request header
        req_header = HeaderCommand()
        memset(req_header.byte, 0, sizeof(req_header.byte))

        b_str_robot_name = self.robot_name.encode('ascii')
        memmove(req_header.val.robotName, c_char_p(b_str_robot_name), len(self.robot_name))

        b_str_robot_ver = self.robot_version.encode('ascii')
        memmove(req_header.val.robotVersion, c_char_p(b_str_robot_ver), len(self.robot_version))

        req_header.val.stepInfo = self.__step_ver
        req_header.val.sof = self.__sof_client

        req_header.val.cmdId = CMD_FOR_EXTENDED
        req_header.val.dataSize = 8

        self.v_invokeId += 1
        req_header.val.invokeId = self.v_invokeId

        # Make request data
        req_data = Data()
        req_data.int2dArr[0] = np.array(ext_cmd)
        req_data.int2dArr[1] = np.array(req_ext_data_size)
        req_data_size = req_header.val.dataSize

        # Send packet to socket
        memmove(write_buffer, req_header.byte, SIZE_HEADER_COMMAND)
        self._send_message(write_buffer, SIZE_HEADER_COMMAND)
        memmove(write_buffer, req_data.byte, req_data_size)
        self._send_message(write_buffer, req_data_size)

        # Send extended packet to socket
        if req_ext_data_size > 0:
            self._send_message(req_ext_data, req_ext_data_size)

        # Recv header from socket
        res_header = HeaderCommand()
        read_buffer = self._recv_message(read_buffer, SIZE_HEADER_COMMAND)
        memmove(res_header.byte, read_buffer, SIZE_HEADER_COMMAND)


        # Recv data from socket
        res_data = Data()
        res_data_size = res_header.val.dataSize
        if res_data_size > SIZE_DATA_TCP_MAX or res_data_size < 0:
            self.__lock.release()
            self.disconnect()
        elif res_data_size > 0:
            read_buffer = self._recv_message(read_buffer, res_data_size)
            memmove(res_data.byte, read_buffer, res_data_size)

        # Check header and error
        ret = self.check_header(req_header, res_header, res_data)

        # Recv extended data from socket
        res_ext_data = Data()
        res_ext_data_size = res_data.int2dArr[1]

        if res_ext_data_size < 0 or res_ext_data_size > sys.maxsize:
            self.__lock.release()
            self.disconnect()
            print("Recv data error: size")
        elif res_data.int2dArr[0] is not ext_cmd:
            self.__lock.release()
            self.disconnect()
            print("Recv data error: ext_cmd {}/{}".format(res_data.int2dArr[0], ext_cmd))
        if res_ext_data_size > 0:
            self._recv_message(res_ext_data, res_ext_data_size)

        self.__lock.release()
        if not ret:
            return ret
        else:
            return ret, res_data, res_data_size


    ############################################################################
    ## Robot command function (Check all)                                     #
    ############################################################################
    def check(self):
        self._handle_command(CMD_CHECK)
    def emergency_stop(self):
        self._handle_command(CMD_EMERGENCY_STOP)
    def reset_robot(self):
        #print("RESET ROBOT")
        self._handle_command(CMD_RESET_ROBOT)
    def set_servo_on_off(self, val_arr):
        _req_data = Data()
        _req_data_size = self.joint_dof
        for j in range(0, self.joint_dof):
            _req_data.bool6dArr[j] = val_arr[j]
        self._handle_command(CMD_SET_SERVO, _req_data, _req_data_size)
    def set_brake_on_off(self, val_arr):
        _req_data = Data()
        _req_data_size = self.joint_dof
        for j in range(0, self.joint_dof):
            _req_data.bool6dArr[j] = val_arr[j]
        self._handle_command(CMD_SET_BRAKE, _req_data, _req_data_size)
    def stop_motion(self):
        self._handle_command(CMD_STOP)
    def execute_move_command(self, cmd_name):
        _req_data = cmd_name.encode('ascii')
        _req_data_size = len(cmd_name)
        self._handle_command(CMD_MOVE, _req_data, _req_data_size)
    def go_home(self):
        self._handle_command(CMD_MOVE_HOME)
    def go_zero(self):
        self._handle_command(CMD_MOVE_ZERO)
    def joint_move_to(self, q):
        _req_data = Data()
        _req_data_size = self.joint_dof*8
        for j in range(0, self.joint_dof):
            _req_data.double6dArr[j] = q[j]
        self._handle_command(CMD_JOINT_MOVE_TO, _req_data, _req_data_size)
    def joint_move_by(self, q):
        _req_data = Data()
        _req_data_size = self.joint_dof * 8
        for j in range(0, self.joint_dof):
            _req_data.double6dArr[j] = q[j]
        self._handle_command(CMD_JOINT_MOVE_BY, _req_data, _req_data_size)
    def task_move_to(self, p):
        print("TASK  MOVE TO")
        _req_data = Data()
        _req_data_size = self.joint_dof * 8
        for j in range(0, self.joint_dof):
            _req_data.double6dArr[j] = p[j]
        self._handle_command(CMD_TASK_MOVE_TO, _req_data, _req_data_size)
    def task_move_by(self, p):

        _req_data = Data()
        _req_data_size = self.joint_dof * 8
        for j in range(0, self.joint_dof):
            _req_data.double6dArr[j] = p[j]
        self._handle_command(CMD_TASK_MOVE_BY, _req_data, _req_data_size)
    def start_current_program(self):
        self._handle_command(CMD_START_CURRENT_PROGRAM)
    def pause_current_program(self):
        self._handle_command(CMD_PAUSE_CURRENT_PROGRAM)
    def resume_current_program(self):
        self._handle_command(CMD_RESUME_CURRENT_PROGRAM)
    def stop_current_program(self):
        self._handle_command(CMD_STOP_CURRENT_PROGRAM)

    def start_default_program(self):
        self._handle_command(CMD_START_DEFAULT_PROGRAM)
    def set_default_program(self, idx):
        _req_data = Data()
        _req_data_size = 4
        _req_data.intVal = idx
        self._handle_command(CMD_REGISTER_DEFAULT_PROGRAM_IDX, _req_data, _req_data_size)
    def get_default_program_idx(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_REGISTERED_DEFAULT_PROGRAM_IDX)
        if not error_code: return np.array(_res_data.intVal)

    # Robot status: all checked
    def is_robot_running(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_IS_ROBOT_RUNNING)
        if not error_code: return np.array(_res_data.boolVal)
    def is_robot_ready(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_IS_READY)
        if not error_code: return np.array(_res_data.boolVal)
    def is_emergency_stop(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_IS_EMG)
        if not error_code: return np.array(_res_data.boolVal)
    def is_collided(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_IS_COLLIDED)
        if not error_code: return np.array(_res_data.boolVal)
    def is_error_state(self):
        #error_code, _res_data, _res_data_size = self._handle_command(CMD_IS_ERR)
        #if not error_code: return np.array(_res_data.boolVal)
        pass
    def is_busy(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_IS_BUSY)
        if not error_code: return np.array(_res_data.boolVal)
    def is_move_finished(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_IS_MOVE_FINISEHD)
        if not error_code: return np.array(_res_data.boolVal)
    def is_home(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_IS_HOME)
        if not error_code: return np.array(_res_data.boolVal)
    def is_zero(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_IS_ZERO)
        if not error_code: return np.array(_res_data.boolVal)
    def is_in_resetting(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_IS_IN_RESETTING)
        if not error_code: return np.array(_res_data.boolVal)
    def is_direct_teaching_mode(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_IS_DIRECT_TECAHING)
        if not error_code: return np.array(_res_data.boolVal)
    def is_teaching_mode(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_IS_TEACHING)
        if not error_code: return np.array(_res_data.boolVal)
    def is_program_running(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_IS_PROGRAM_RUNNING)
        if not error_code: return np.array(_res_data.boolVal)
    def is_program_paused(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_IS_PROGRAM_PAUSED)
        if not error_code: return np.array(_res_data.boolVal)
    def is_conty_connected(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_IS_CONTY_CONNECTED)
        if not error_code: return np.array(_res_data.boolVal)

    # Direct teaching
    def change_to_direct_teaching(self):
        self._handle_command(CMD_CHANGE_DIRECT_TEACHING)
    def finish_direct_teaching(self):
        self._handle_command(CMD_FINISH_DIRECT_TEACHING)

    # Simple waypoint program, joint and task
    def add_joint_waypoint(self, q):
        _req_data = Data()
        _req_data_size = self.joint_dof * 8
        for j in range(0, self.joint_dof):
            _req_data.double6dArr[j] = q[j]
        self._handle_command(CMD_JOINT_PUSH_BACK_WAYPOINT_SET, _req_data, _req_data_size)
    def remove_last_joint_waypoint(self):
        self._handle_command(CMD_JOINT_POP_BACK_WAYPOINT_SET)
    def clear_joint_waypoints(self):
        self._handle_command(CMD_JOINT_CLEAR_WAYPOINT_SET)
    def execute_joint_waypoints(self):
        self._handle_command(CMD_JOINT_EXECUTE_WAYPOINT_SET)
    def add_task_waypoint(self, p):
        _req_data = Data()
        _req_data_size = 48
        for j in range(0, 6):
            _req_data.double6dArr[j] = p[j]
        self._handle_command(CMD_TASK_PUSH_BACK_WAYPOINT_SET, _req_data, _req_data_size)
    def remove_last_task_waypoint(self):
        self._handle_command(CMD_TASK_POP_BACK_WAYPOINT_SET)
    def clear_task_waypoints(self):
        self._handle_command(CMD_TASK_CLEAR_WAYPOINT_SET)
    def execute_task_waypoints(self):
        self._handle_command(CMD_TASK_EXECUTE_WAYPOINT_SET)

    # Get/Set some global robot variables
    def set_default_tcp(self, tcp):
        _req_data = Data()
        _req_data_size = 48
        for j in range(0, 6):
            _req_data.double6dArr[j] = tcp[j]
        self._handle_command(CMD_SET_DEFAULT_TCP, _req_data, _req_data_size)
    def reset_default_tcp(self):
        self._handle_command(CMD_RESET_DEFAULT_TCP)
    def set_tcp_compensation(self, tcp):
        _req_data = Data()
        _req_data_size = 48
        for j in range(0, 6):
            _req_data.double6dArr[j] = tcp[j]
        self._handle_command(CMD_SET_COMP_TCP, _req_data, _req_data_size)
    def reset_tcp_compensation(self):
        self._handle_command(CMD_RESET_COMP_TCP)
    def set_ref_frame(self, ref):
        _req_data = Data()
        _req_data_size = 48
        for j in range(0, 6):
            _req_data.double6dArr[j] = ref[j]
        self._handle_command(CMD_SET_REFFRAME, _req_data, _req_data_size)
    def reset_ref_frame(self):
        self._handle_command(CMD_RESET_REFFRAME)

    def set_collision_level(self, level):
        _req_data = Data()
        _req_data_size = 4
        val = level
        min_val = 1
        max_val = 5
        val = min_val if val < min_val else max_val if val > max_val else val
        _req_data.intVal = val
        self._handle_command(CMD_SET_COLLISION_LEVEL, _req_data, _req_data_size)
    def set_joint_boundary_level(self, level):
        _req_data = Data()
        _req_data_size = 4
        val = level
        min_val = 1
        max_val = 9
        val = min_val if val < min_val else max_val if val > max_val else val
        _req_data.intVal = val
        self._handle_command(CMD_SET_JOINT_BOUNDARY, _req_data, _req_data_size)
    def set_task_boundary_level(self, level):
        _req_data = Data()
        _req_data_size = 4
        val = level
        min_val = 1
        max_val = 9
        val = min_val if val < min_val else max_val if val > max_val else val
        _req_data.intVal = val
        self._handle_command(CMD_SET_TASK_BOUNDARY, _req_data, _req_data_size)

    def set_joint_waypoint_time(self, time):
        _req_data = Data()
        _req_data_size = 8
        val = time
        min_val = 0
        max_val = 10
        val = min_val if val < min_val else max_val if val > max_val else val
        _req_data.doubleVal = val
        self._handle_command(CMD_SET_JOINT_WTIME, _req_data, _req_data_size)
    def set_task_waypoint_time(self, time):
        _req_data = Data()
        _req_data_size = 8
        val = time
        min_val = 0
        max_val = 10
        val = min_val if val < min_val else max_val if val > max_val else val
        _req_data.doubleVal = val
        self._handle_command(CMD_SET_TASK_WTIME, _req_data, _req_data_size)

    def set_task_base_mode(self, mode):  # Not work?
        # 0: reference body, 1: end-effector tool tip
        _req_data = Data()
        _req_data_size = 4
        _req_data.intVal = mode
        self._handle_command(CMD_SET_TASK_CMODE, _req_data, _req_data_size)

    def set_joint_blend_radius(self, radius):
        _req_data = Data()
        _req_data_size = 4
        val = radius
        min_val = 8
        max_val = 27
        val = min_val if val < min_val else max_val if val > max_val else val
        _req_data.doubleVal = val
        self._handle_command(CMD_SET_JOINT_BLEND_RADIUS, _req_data, _req_data_size)
    def set_task_blend_radius(self, radius):
        _req_data = Data()
        _req_data_size = 8
        val = radius
        min_val = 0.02
        max_val = 0.2
        val = min_val if val < min_val else max_val if val > max_val else val
        _req_data.doubleVal = val
        self._handle_command(CMD_SET_TASK_BLEND_RADIUS, _req_data, _req_data_size)

    def get_default_tcp(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_DEFAULT_TCP)
        if not error_code: return np.array(_res_data.double6dArr)
    def get_tcp_compensation(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_COMP_TCP)
        if not error_code: return np.array(_res_data.double6dArr)

    def get_ref_frame(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_REFFRAME)
        if not error_code:
            return np.array(_res_data.double6dArr)
    def get_collision_level(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_COLLISION_LEVEL)
        if not error_code:
            return np.array(_res_data.intVal)

    def get_joint_boundary_level(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_JOINT_BOUNDARY)
        if not error_code:
            return np.array(_res_data.intVal)
    def get_task_boundary_level(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_TASK_BOUNDARY)
        if not error_code:
            return np.array(_res_data.intVal)

    def get_joint_waypoint_time(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_JOINT_WTIME)
        if not error_code:
            return np.array(_res_data.doubleVal)
    def get_task_waypoint_time(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_TASK_WTIME)
        if not error_code:
            return np.array(_res_data.doubleVal)

    def get_task_base_mode(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_TASK_CMODE)
        if not error_code:
            return np.array(_res_data.intVal)
    def get_joint_blend_radius(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_JOINT_BLEND_RADIUS)
        if not error_code:
            return np.array(_res_data.doubleVal)
    def get_task_blend_radius(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_TASK_BLEND_RADIUS)
        if not error_code:
            return np.array(_res_data.doubleVal)
    def get_robot_running_time(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_RUNNING_TIME)
        if not error_code:
            return np.array(_res_data.doubleVal)
    def get_cmode(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_CMODE)
        if not error_code:
            return np.array(_res_data.intVal)

    def get_joint_servo_state(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_JOINT_STATE)
        if not error_code:
            result = np.array(_res_data.charArr)
            servo_state = result[0:self.joint_dof]
            brake_state = result[self.joint_dof:2*self.joint_dof]
            return servo_state, brake_state

    def get_joint_pos(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_JOINT_POSITION)
        if not error_code:
            if self.joint_dof == 6: return np.array(_res_data.double6dArr)
            elif self.joint_dof == 7: return np.array(_res_data.double7dArr)

    def get_joint_vel(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_JOINT_VELOCITY)
        if not error_code:
            if self.joint_dof == 6: return np.array(_res_data.double6dArr)
            elif self.joint_dof == 7: return np.array(_res_data.double7dArr)

    def get_task_pos(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_TASK_POSITION)
        if not error_code: return np.array(_res_data.double6dArr)

    def get_task_vel(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_TASK_VELOCITY)
        if not error_code: return np.array(_res_data.double6dArr)

    def get_torque(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_TORQUE)
        if not error_code: return np.array(_res_data.double6dArr)

    def get_last_emergency_info(self): # Check (TODO: represent meaning of results)
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_LAST_EMG_INFO)
        if not error_code:
            ret_code = c_int32()
            ret_int_arr = (c_int32 * 3)()
            ret_double_arr = (c_double*3)()

            memmove(addressof(ret_code), addressof(_res_data.byte), 4)
            memmove(addressof(ret_int_arr), addressof(_res_data.byte) + 4, 4 * 3)
            memmove(addressof(ret_double_arr), addressof(_res_data.byte) + 16, 8 * 3)

            return np.array(ret_code), np.array(ret_int_arr), np.array(ret_double_arr)

    # I/O
    def get_smart_di(self, idx):
        _req_data = Data()
        _req_data_size = 4
        _req_data.intVal = idx

        _res_data = Data()
        _res_data_size = 1
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_SMART_DI, _req_data, _req_data_size)
        if not error_code: return np.array(_res_data.charVal)

    def get_smart_dis(self):
        _res_data = Data()
        _res_data_size = 32
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_SMART_DIS)
        if not error_code:
            return np.array(_res_data.charArr)

    def set_smart_do(self, idx, val):
        _req_data = Data()
        _req_data_size = 5

        memset(_req_data.byte, 0, sizeof(_req_data.byte))
        memmove(_req_data.byte, pointer(c_int32(idx)), sizeof(c_int32))
        memmove(addressof(_req_data.byte)+4, pointer(c_ubyte(val)), sizeof(c_ubyte))

        self._handle_command(CMD_SET_SMART_DO, _req_data, _req_data_size)

    def set_smart_dos(self, np_arr):
        _req_data = Data()
        _req_data_size = 32
        for i in range(0, _req_data_size):
            _req_data.charArr[i] = np_arr[i]

        self._handle_command(CMD_SET_SMART_DOS, _req_data, _req_data_size)

    def get_smart_ai(self, idx):
        _req_data = Data()
        _req_data_size = 4
        _req_data.intVal = idx

        _res_data = Data()
        _res_data_size = 4
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_SMART_AI, _req_data, _req_data_size)
        if not error_code:
            return np.array(_res_data.intVal)

    def set_smart_ao(self, idx, val):
        _req_data = Data()
        _req_data_size = 8
        _req_data.intArr[0] = idx
        _req_data.intArr[1] = val
        self._handle_command(CMD_SET_SMART_AO, _req_data, _req_data_size)

    def get_smart_do(self, idx):
        _req_data = Data()
        _req_data_size = 4
        _req_data.intVal = idx

        _res_data = Data()
        _res_data_size = 1
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_SMART_DO, _req_data, _req_data_size)
        if not error_code:
            return np.array(_res_data.charVal)

    def get_smart_dos(self):
        _res_data_size = 32
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_SMART_DOS)
        if not error_code:
            return np.array(_res_data.charArr)

    def get_smart_ao(self, idx):
        _req_data = Data()
        _req_data_size = 4
        _req_data.intVal = idx

        _res_data_size = 4
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_SMART_AO, _req_data, _req_data_size)
        if not error_code:
            return np.array(_res_data.intVal)

    # Not yet implemented
    def get_robot_canft_sensor_raw(self):
        pass
    def get_robot_canft_sensor_process(self):
        pass
    def get_cb_canft_sensor_raw(self):
        pass
    def get_cb_canft_sensor_process(self):
        pass

    def read_direct_variable(self):
        pass
    def read_direct_variables(self):
        pass
    def write_direct_variable(self):
        pass
    def write_direct_variables(self):
        pass

    ############################################################################
    ## Extended IndyDCP command (Check all)                                    #
    ############################################################################
    def move_ext_traj_text_file(self, file_name):  # Check
        file_name += "\0"  # last char should be null
        req_ext_data = file_name.encode('ascii')
        req_ext_data_size = len(file_name)
        self._handle_extended_command(EXT_CMD_MOVE_TRAJ_BY_TXT_FILE,
                                              req_ext_data,
                                              req_ext_data_size)

    ############################################################################
    ## JSON programming added (only for internal engineer)                     #
    ############################################################################
    def set_json_program(self):
        pass

    def set_and_start_json_program(self, json_string):
        json_string += "\0"
        req_ext_data = json_string.encode('ascii')
        req_ext_data_size = len(json_string)
        self._handle_extended_command(EXT_CMD_SET_JSON_PROG_START,
                                      req_ext_data,
                                      req_ext_data_size)

###############################################################################
# Test                                                                        #
###############################################################################
if __name__ =='__main__':
    if len(sys.argv)<4:
        print('{0} <Bind IP> <Server IP> <Robot Name>'.format(sys.argv[0]))
        sys.exit()

    _bind_ip = sys.argv[1]
    _server_ip = sys.argv[2]
    _name = sys.argv[3]

    # Connect
    indy= IndyDCPClient(_bind_ip, _server_ip, _name)
    indy.connect()
    print('connection to ', _server_ip, ' : ', indy.is_connected())

    # Check robot ready
    print('### Test: IsReady() ###')
    if indy.is_robot_ready():
        print('Robot is ready!')
    else:
        print('Robot is not ready!')

    # Check moving finished
    print('### Test: IsMoveFinished() ###')
    if indy.is_move_finished():
        print('Robot is not moving!')
    else:
        print('Robot is moving!')

    # Check DirectTeaching
    print('### Test: StartDirectTeaching() ###')
    if indy.change_to_direct_teaching() :
        print('Start DirectTeaching success!')
    else:
        print('Start DirectTeaching failed!')

    print('### Test: StopDirectTeaching() ###')
    if indy.finish_direct_teaching() :
        print('Stop DirectTeaching success!')
    else:
        print('Stop DirectTeaching failed!')

    # Get Task Position
    print('### Test: GetTaskPos() ###')
    task_pos = indy.get_task_pos()
    print ("Task Pos: ")
    print (task_pos)

    # Get Joint Position
    print('### Test: GetJointPos() ###')
    joint_pos = indy.get_joint_pos()
    print ("Joint Pos: ")
    print (joint_pos)

    # Move to Task
    print('### Test: MoveToT() ###')
    indy.task_move_to(task_pos)

    # Move to Joint
    print('### Test: MoveToJ() ###')
    indy.joint_move_to(joint_pos)
    # Disconnect
    indy.disconnect()
    print("Test finished")
