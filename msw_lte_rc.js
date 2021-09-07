/**
 * Created by Wonseok Jung in KETI on 2021-07-01.
 */

// for TAS of mission

let mqtt = require('mqtt');
let fs = require('fs');

let my_msw_name = 'msw_lte_rc';

let fc = {};
let config = {};

global.libPort = 0;
global.libBaudrate = 0;

try {                                   // for nCube-MUV (NodeJs)
    sortie_name = my_sortie_name
} catch (e) {                             // for nCube-MUV-Python
    sortie_name = process.argv[2]
}

config.name = my_msw_name;

try {
    if (drone_info !== null) {  // for nCube-MUV (NodeJs)
        config.directory_name = msw_directory[my_msw_name];
        config.sortie_name = '/' + sortie_name;
        config.gcs = drone_info.gcs;
        config.drone = drone_info.drone;
        config.lib = [];
    } else if (process.argv[5] !== null) {     // for nCube-MUV-Python
        config.directory_name = process.argv[3];
        config.sortie_name = '/' + sortie_name;
        config.gcs = process.argv[4];
        config.drone = process.argv[5];
        config.lib = [];
    }
} catch (e) {
    config.sortie_name = '';
    config.directory_name = '';
    config.gcs = 'KETI_MUV';
    config.drone = 'KETI_Demo';
    config.lib = [];
}

let add_lib = {};
try {
    add_lib = JSON.parse(fs.readFileSync('./' + config.directory_name + '/lib_lte_rc.json', 'utf8'));
    config.lib.push(add_lib);
} catch (e) {
    add_lib = {
        name: 'lib_lte_rc',
        target: 'armv6',
        description: "node [name] [portnum] [baudrate]",
        scripts: 'node lib_lte_rc /dev/ttyUSB3 115200',
        data: ['SBUS'],
        control: ['REMOTE', 'STATUS']
    };
    config.lib.push(add_lib);
}
// msw가 muv로 부터 트리거를 받는 용도
// 명세에 sub_container 로 표기
let msw_sub_muv_topic = [];

let msw_sub_fc_topic = [];
msw_sub_fc_topic.push('/Mobius/' + config.gcs + '/Drone_Data/' + config.drone + '/heartbeat');
msw_sub_fc_topic.push('/Mobius/' + config.gcs + '/Drone_Data/' + config.drone + '/global_position_int');
msw_sub_fc_topic.push('/Mobius/' + config.gcs + '/Drone_Data/' + config.drone + '/attitude');
msw_sub_fc_topic.push('/Mobius/' + config.gcs + '/Drone_Data/' + config.drone + '/battery_status');

let msw_sub_lib_topic = [];
let remote_topic = '';
let status_topic = '';
remote_topic = '/Mobius/' + config.gcs + '/Mission_Data/' + config.drone + '/' + config.name + '/' + config.lib[0].control[0];
status_topic = '/Mobius/' + config.gcs + '/Mission_Data/' + config.drone + '/' + config.name + '/' + config.lib[0].control[1];

let muv_sub_gcs_topic = '/Mobius/' + config.gcs + '/GCS_Data/' + config.drone;

const joystick_params = ['RC1_MAX', 'RC1_MIN', 'RC1_TRIM', 'RC1_REVERSED', 'RC2_MAX', 'RC2_MIN', 'RC2_TRIM', 'RC2_REVERSED', 'RC3_MAX', 'RC3_MIN', 'RC3_TRIM', 'RC3_REVERSED', 'RC4_MAX', 'RC4_MIN', 'RC4_TRIM', 'RC4_REVERSED', 'RC5_MAX', 'RC5_MIN', 'RC5_TRIM', 'RC6_MAX', 'RC6_MIN', 'RC6_TRIM']

function mavlinkGenerateMessage(src_sys_id, src_comp_id, type, params) {
    const mavlinkParser = new MAVLink(null/*logger*/, src_sys_id, src_comp_id);
    let genMsg = null;
    let mavMsg = null;

    try {
        switch (type) {
            case mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                mavMsg = new mavlink.messages.param_request_read(params.target_system,
                    params.target_component,
                    params.param_id,
                    params.param_index);
                break;
            case mavlink.MAVLINK_MSG_ID_PARAM_SET:
                mavMsg = new mavlink.messages.param_set(params.target_system,
                    params.target_component,
                    params.param_id,
                    params.param_value,
                    params.param_type);
                break;
        }
    } catch (e) {
        console.log('MAVLINK EX:' + e);
    }

    if (mavMsg) {
        genMsg = Buffer.from(mavMsg.pack(mavlinkParser));
    }

    return genMsg;
}

function set_rc_reversed_param(target_name, pub_topic, target_sys_id, param_id) {
    let btn_params = {};
    btn_params.target_system = target_sys_id;
    btn_params.target_component = 1;
    btn_params.param_id = param_id;
    btn_params.param_type = mavlink.MAV_PARAM_TYPE_INT8;
    btn_params.param_value = 0;
    try {
        let msg = mavlinkGenerateMessage(255, 0xbe, mavlink.MAVLINK_MSG_ID_PARAM_SET, btn_params);
        if (msg == null) {
            console.log("mavlink message is null");
        } else {
            MSW_mobius_mqtt_client.publish(pub_topic, msg);
        }
        console.log(msg);
    } catch (ex) {
        console.log('[ERROR] ' + ex);
    }
}

function send_param_get_command(target_name, pub_topic, target_sys_id, param_id) {
    let btn_params = {};
    btn_params.target_system = target_sys_id;
    btn_params.target_component = 1;
    btn_params.param_id = param_id;
    btn_params.param_index = -1;

    try {
        let msg = mavlinkGenerateMessage(255, 0xbe, mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_READ, btn_params);
        if (msg == null) {
            console.log("mavlink message is null");
        } else {
            MSW_mobius_mqtt_client.publish(pub_topic, msg);
        }
    } catch (ex) {
        console.log('[ERROR] ' + ex);
    }
}

let command_delay = 0;
let status_flag = 0;  // 0 does not subscribe to remote topic, 1 subscribes to remote topic.

function init() {
    if (config.lib.length > 0) {
        for (let idx in config.lib) {
            if (config.lib.hasOwnProperty(idx)) {
                // Request RC PARAMs
                for (let param_idx in joystick_params) {
                    if (joystick_params.hasOwnProperty(param_idx)) {
                        command_delay++;
                        setTimeout(send_param_get_command, command_delay, config.drone, muv_sub_gcs_topic, drone_info.system_id, joystick_params[param_idx]);
                    }
                }
                setTimeout(function () {
                    for (let param_idx in joystick_params) {
                        if (rc_map.hasOwnProperty(joystick_params[param_idx].toLowerCase())) {
                            // console.log(util.format('rc_map.[%s] = %d', joystick_params[param_idx].toLowerCase(), rc_map[joystick_params[param_idx].toLowerCase()]));
                        } else {
                            // console.log('one more send req message');
                            command_delay++;
                            setTimeout(send_param_get_command, command_delay, config.drone, muv_sub_gcs_topic, drone_info.system_id, joystick_params[param_idx]);
                        }
                    }

                    if (rc_map.rc1_reversed !== 0) {
                        set_rc_reversed_param(config.drone, muv_sub_gcs_topic, drone_info.system_id, 'RC1_REVERSED');
                        send_param_get_command(config.drone, muv_sub_gcs_topic, drone_info.system_id, 'RC1_REVERSED');
                    } else {
                    }
                    if (rc_map.rc2_reversed !== 0) {
                        set_rc_reversed_param(config.drone, muv_sub_gcs_topic, drone_info.system_id, 'RC2_REVERSED');
                        send_param_get_command(config.drone, muv_sub_gcs_topic, drone_info.system_id, 'RC2_REVERSED');
                    } else {
                    }
                    if (rc_map.rc3_reversed !== 0) {
                        set_rc_reversed_param(config.drone, muv_sub_gcs_topic, drone_info.system_id, 'RC3_REVERSED');
                        send_param_get_command(config.drone, muv_sub_gcs_topic, drone_info.system_id, 'RC3_REVERSED');
                    } else {
                    }
                    if (rc_map.rc4_reversed !== 0) {
                        set_rc_reversed_param(config.drone, muv_sub_gcs_topic, drone_info.system_id, 'RC4_REVERSED');
                        send_param_get_command(config.drone, muv_sub_gcs_topic, drone_info.system_id, 'RC4_REVERSED');
                    } else {
                    }
                }, 1000);

                if (msw_mqtt_client != null) {
                    for (let i = 0; i < config.lib[idx].control.length; i++) {
                        let sub_container_name = config.lib[idx].control[i];
                        let _topic = '/Mobius/' + config.gcs + '/Mission_Data/' + config.drone + '/' + config.name + '/' + sub_container_name;
                        msw_mqtt_client.subscribe(_topic);
                        msw_sub_muv_topic.push(_topic);
                        console.log('[msw_mqtt] msw_sub_muv_topic[' + i + ']: ' + _topic);
                    }

                    for (let i = 0; i < config.lib[idx].data.length; i++) {
                        let container_name = config.lib[idx].data[i];
                        let _topic = '/MUV/data/' + config.lib[idx].name + '/' + container_name;
                        msw_mqtt_client.subscribe(_topic);
                        msw_sub_lib_topic.push(_topic);
                        console.log('[lib_mqtt] lib_topic[' + i + ']: ' + _topic);
                    }
                }

                let obj_lib = config.lib[idx];
                setTimeout(runLib, 1000 + parseInt(Math.random() * 10), JSON.parse(JSON.stringify(obj_lib)));
            }
        }
    }
}

function runLib(obj_lib) {
    try {
        let scripts_arr = obj_lib.scripts.split(' ');

        libPort = scripts_arr[2];
        libBaudrate = scripts_arr[3];

        if (config.directory_name === '') {

        } else {
            scripts_arr[0] = scripts_arr[0].replace('./', '');
            scripts_arr[0] = './' + config.directory_name + '/' + scripts_arr[0];
        }

        require('./lib_lte_rc');
    } catch (e) {
        console.log(e.message);
    }
}

let msw_mqtt_client = null;

msw_mqtt_connect('localhost', 1883);

function msw_mqtt_connect(broker_ip, port) {
    if (msw_mqtt_client == null) {
        let connectOptions = {
            host: broker_ip,
            port: port,
            protocol: "mqtt",
            keepalive: 10,
            protocolId: "MQTT",
            protocolVersion: 4,
            clean: true,
            reconnectPeriod: 2000,
            connectTimeout: 2000,
            rejectUnauthorized: false
        };

        msw_mqtt_client = mqtt.connect(connectOptions);
    }

    msw_mqtt_client.on('connect', function () {
        console.log('[msw_mqtt_connect] connected to ' + broker_ip);
        for (idx in msw_sub_fc_topic) {
            if (msw_sub_fc_topic.hasOwnProperty(idx)) {
                msw_mqtt_client.subscribe(msw_sub_fc_topic[idx]);
                console.log('[msw_mqtt_connect] msw_sub_fc_topic[' + idx + ']: ' + msw_sub_fc_topic[idx]);
            }
        }
    });

    msw_mqtt_client.on('message', function (topic, message) {
        for (idx in msw_sub_lib_topic) {
            if (msw_sub_lib_topic.hasOwnProperty(idx)) {
                if (topic === msw_sub_lib_topic[idx]) {
                    setTimeout(on_receive_from_lib, parseInt(Math.random() * 5), topic, message.toString());
                    break;
                }
            }
        }

        for (idx in msw_sub_fc_topic) {
            if (msw_sub_fc_topic.hasOwnProperty(idx)) {
                if (topic === msw_sub_fc_topic[idx]) {
                    setTimeout(on_process_fc_data, parseInt(Math.random() * 5), topic, message.toString());
                    break;
                }
            }
        }

        if (topic === status_topic) {
            if (message.toString() === 'ON') {
                if (status_flag === 0) {
                    msw_mqtt_client.subscribe(remote_topic);
                    console.log('[msw_mqtt_connect] remote_topic : ' + remote_topic);
                    status_flag = 1;
                } else {
                }
            } else if (message.toString() === 'OFF') {
                if (status_flag === 1) {
                    msw_mqtt_client.unsubscribe(remote_topic);
                    console.log('[msw_mqtt_connect] remote_topic : ' + remote_topic);
                    status_flag = 0;
                } else {
                }
            }
        } else if (topic === remote_topic) {
            on_receive_from_muv(topic, message.toString());
        }
    });

    msw_mqtt_client.on('error', function (err) {
        console.log(err.message);
    });
}

function on_receive_from_muv(topic, str_message) {
    // console.log('[' + topic + '] ' + str_message);

    parseControlMission(topic, str_message);
}

function on_receive_from_lib(topic, str_message) {
    // console.log('[' + topic + '] ' + str_message);

    parseDataMission(topic, str_message);
}

function on_process_fc_data(topic, str_message) {
    let topic_arr = topic.split('/');
    fc[topic_arr[topic_arr.length - 1]] = JSON.parse(str_message.toString());

    parseFcData(topic, str_message);
}

setTimeout(init, 1000);

function parseDataMission(topic, str_message) {
    try {
        // User define Code
        // let obj_lib_data = JSON.parse(str_message);
        //
        // if(fc.hasOwnProperty('global_position_int')) {
        //     Object.assign(obj_lib_data, JSON.parse(JSON.stringify(fc['global_position_int'])));
        // }
        // str_message = JSON.stringify(obj_lib_data);

        ///////////////////////////////////////////////////////////////////////
        let topic_arr = topic.split('/');
        let data_topic = '/Mobius/' + config.gcs + '/Mission_Data/' + config.drone + '/' + config.name + '/' + topic_arr[topic_arr.length - 1];
        msw_mqtt_client.publish(data_topic + '/' + sortie_name, str_message);
    } catch (e) {
        console.log('[parseDataMission] data format of lib is not json');
    }
}

///////////////////////////////////////////////////////////////////////////////

function parseControlMission(topic, str_message) {
    try {
        // User define Code
        ///////////////////////////////////////////////////////////////////////

        let topic_arr = topic.split('/');
        let _topic = '/MUV/control/' + config.lib[0].name + '/' + topic_arr[topic_arr.length - 1];
        msw_mqtt_client.publish(_topic, str_message);
    } catch (e) {
        console.log('[parseControlMission] data format of MUV is not json');
    }
    // let indata = JSON.parse(str_message);

    // console.log('[' + topic + '] ' + indata.con);
    // console.log('msw message received from nCube');
    // let container_name = config.lib[0].control[0];
    // let control_topic = '/MUV/control/' + config.lib[0].name + '/' + container_name;
    // console.log('topic: ' + topic + ' cmd: ' + str_message);
    // msw_mqtt_client.publish(control_topic, str_message);

}

function parseFcData(topic, str_message) {
    // User define Code
    // let topic_arr = topic.split('/');
    // if(topic_arr[topic_arr.length-1] == 'global_position_int') {
    //     let _topic = '/MUV/control/' + config.lib[0].name + '/' + config.lib[1].control[1]; // 'Req_enc'
    //     msw_mqtt_client.publish(_topic, str_message);
    // }
    ///////////////////////////////////////////////////////////////////////
}


let MSW_mobius_mqtt_client = null;

MSW_mobius_mqtt_connect(drone_info.host, 1883);

function MSW_mobius_mqtt_connect(broker_ip, port) {
    if (MSW_mobius_mqtt_client == null) {
        let connectOptions = {
            host: broker_ip,
            port: port,
            protocol: "mqtt",
            keepalive: 10,
            protocolId: "MQTT",
            protocolVersion: 4,
            clean: true,
            reconnectPeriod: 2000,
            connectTimeout: 2000,
            rejectUnauthorized: false
        };

        MSW_mobius_mqtt_client = mqtt.connect(connectOptions);
        MSW_mobius_mqtt_client.on('connect', function () {
            console.log('[msw_mobius_mqtt_connect] connected to ' + broker_ip);
            if (status_topic !== '') {
                MSW_mobius_mqtt_client.subscribe(status_topic);
                console.log('[msw_mobius_mqtt_subscribe] status_topic : ' + status_topic);
            }
            if (remote_topic !== '') {
                MSW_mobius_mqtt_client.subscribe(remote_topic);
                status_flag = 1;
                console.log('[msw_mobius_mqtt_subscribe] remote_topic : ' + remote_topic);
            }
        });

        MSW_mobius_mqtt_client.on('message', function (topic, message) {
            if (topic === status_topic) {
                if (message.toString() === 'ON') {
                    if (status_flag === 0) {
                        MSW_mobius_mqtt_client.subscribe(remote_topic);
                        console.log('[msw_mobius_mqtt_subscribe] remote_topic : ' + remote_topic);
                        status_flag = 1;
                    } else {
                    }
                } else if (message.toString() === 'OFF') {
                    if (status_flag === 1) {
                        MSW_mobius_mqtt_client.unsubscribe(remote_topic);
                        console.log('[msw_mobius_mqtt_unsubscribe] remote_topic : ' + remote_topic);
                        status_flag = 0;
                    } else {
                    }
                }
            } else if (topic === remote_topic) {
                on_receive_from_muv(topic, message.toString());
            }
        });

        MSW_mobius_mqtt_client.on('error', function (err) {
            console.log(err.message);
        });
    }
}
