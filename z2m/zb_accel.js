const { Buffer } = require('node:buffer');
const util = require('node:util');
const {Zcl} = require('zigbee-herdsman');
const {enumLookup,numeric,deviceAddCustomCluster,onOff,binary, battery} = require('zigbee-herdsman-converters/lib/modernExtend');
const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const constants = require('zigbee-herdsman-converters/lib/constants');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const ota = require('zigbee-herdsman-converters/lib/ota');
const utils = require('zigbee-herdsman-converters/lib/utils');
const globalStore = require('zigbee-herdsman-converters/lib/store');
const {logger} = require('zigbee-herdsman-converters/lib/logger');
const e = exposes.presets;
const eo = exposes.options;
const ea = exposes.access;

const NS = 'zhc:orlangur';

const orlangurAccelExtended = {
    extendedStatus: () => {
        const exposes = [
            e.numeric('status1', ea.STATE_GET).withLabel('Status1').withCategory('diagnostic'),
            e.numeric('status2', ea.STATE_GET).withLabel('Status2').withCategory('diagnostic'),
            e.numeric('status3', ea.STATE_GET).withLabel('Status3').withCategory('diagnostic'),
        ];

        const fromZigbee = [
            {
                cluster: 'customStatus',
                type: ['attributeReport', 'readResponse'],
                convert: (model, msg, publish, options, meta) => {
                    const result = {};
                    if (data['status1'] !== undefined) 
                        result['status1'] = data['status1'];
                    if (data['status2'] !== undefined) 
                        result['status2'] = data['status2'];
                    if (data['status3'] !== undefined) 
                        result['status3'] = data['status3'];
                    return result
                }
            }
        ];

        const toZigbee = [
            {
                key: ['status1', 'status2', 'status3'],
                convertGet: async (entity, key, meta) => {
                    await entity.read('customStatus', [key]);
                },
            }
        ];

        return {
            exposes,
            fromZigbee,
            toZigbee,
            isModernExtend: true,
        };
    },
    accelConfig: () => {
        const sleep_odr = {
            "Same"  : 0,
            "1.6 Hz"  : 1,
            "3 Hz"  : 2,
            "6 Hz"  : 3,
        };

        const active_odr = {
            "Off"         : 0,
            "1.6 Hz ULP"  : 1,
            "3 Hz ULP"    : 2,
            "6 Hz ULP"    : 3,
            "6 Hz"        : 4,
            "12.5 Hz"     : 5,
            "25 Hz"       : 6,
            "50 Hz"       : 7,
            "100 Hz"      : 8,
            "200 Hz"      : 9,
            "400 Hz"      : 10,
            "800 Hz"      : 11,
        };

        const exposes = [
            e.binary('enable_x', ea.ALL, 1, 0).withCategory('config').withDescription('Sensing activity on X'),
            e.binary('enable_y', ea.ALL, 1, 0).withCategory('config').withDescription('Sensing activity on Y'),
            e.binary('enable_z', ea.ALL, 1, 0).withCategory('config').withDescription('Sensing activity on Z'),
            e.binary('track_wake_up', ea.ALL, 1, 0).withCategory('config').withDescription('Track wake-up events'),
            e.binary('track_sleep', ea.ALL, 1, 0).withCategory('config').withDescription('Track sleep events'),
            e.binary('track_flip', ea.ALL, 1, 0).withCategory('config').withDescription('Track flip events'),
            e.numeric('wake_sleep_threshold', ea.ALL)
                .withCategory('config')
                // .withValueMin(0)
                // .withValueMax(100)
                .withDescription('Wake Sleep Threshold')
                .withLabel('Wake Sleep Threshold'),
            e.numeric('sleep_duration', ea.ALL)
                .withCategory('config')
                // .withValueMin(0)
                // .withValueMax(100)
                .withDescription('Sleep Duration')
                .withLabel('Sleep Duration'),
            e.enum('sleep_odr', ea.ALL, Object.keys(sleep_odr))
                .withCategory('config')
                .withDescription('Sleep ODR')
                .withLabel('Sleep ODR'),
            e.enum('active_odr', ea.ALL, Object.keys(active_odr))
                .withCategory('config')
                .withDescription('Active ODR')
                .withLabel('Active ODR'),
        ];

        const cfg_bits = {
            enable_x       : 0,
            enable_y       : 1,
            enable_z       : 2,
            track_wake_up  : 3,
            track_sleep    : 4,
            track_flip     : 5,
        };

        const fromZigbee = [
            {
                cluster: 'customConfig',
                type: ['attributeReport', 'readResponse'],
                convert: (model, msg, publish, options, meta) => {
                    const result = {};
                    const data = msg.data;

                    if (data['flags'] !== undefined) 
                    {
                        const buffer = Buffer.alloc(1);
                        buffer.writeUInt8(data['flags']);
                        const b0 = buffer.readUInt8(0);
                        const set_cfg = (name, b0) => { result[name] = (b0 >> cfg_bits[name]) & 1; };
                        set_cfg('enable_x', b0);
                        set_cfg('enable_y', b0);
                        set_cfg('enable_z', b0);
                        set_cfg('track_wake_up', b0);
                        set_cfg('track_sleep', b0);
                        set_cfg('track_flip', b0);
                    }

                    if (data['wake_sleep_threshold'] !== undefined)
                        result['wake_sleep_threshold'] = data['wake_sleep_threshold'];
                    if (data['sleep_duration'] !== undefined)
                        result['sleep_duration'] = data['sleep_duration'];
                    if (data['sleep_odr'] !== undefined)
                    {
                        const v = data['sleep_odr']
                        const entry = Object.entries(sleep_odr).find(([_,val])=> val == v)
                        if (entry)
                            result['sleep_odr'] = entry[0];//key
                    }
                    if (data['active_odr'] !== undefined)
                    {
                        const v = data['active_odr']
                        const entry = Object.entries(active_odr).find(([_,val])=> val == v)
                        if (entry)
                            result['active_odr'] = entry[0];//key
                    }

                    if (Object.keys(result).length == 0) 
                        return;

                    return result;
                }
            }
        ];

        const toZigbee = [
            {
                key: ['enable_x', 'enable_y', 'enable_z', 'track_wake_up', 'track_sleep', 'track_flip'],
                convertSet: async (entity, key, value, meta) => {
                    //read current state
                    const readResult = await entity.read('customConfig', ['flags']);
                    //update the requested bit
                    const newVal = (readResult.flags & ~(1 << cfg_bits[key])) | (value << cfg_bits[key])
                    await entity.write('customConfig', {['flags']: newVal});
                    return {state: {[key]: value}};
                },
                convertGet: async (entity, key, meta) => {
                    await entity.read('customConfig', ['flags']);
                },
            },
            {
                key: ['wake_sleep_threshold', 'sleep_duration'],
                convertSet: async (entity, key, value, meta) => {
                    await entity.write('customConfig', {[key]: value});
                    return {state: {[key]: value}};
                },
                convertGet: async (entity, key, meta) => {
                    await entity.read('customConfig', [key]);
                },
            }
            ,{
                key: ['sleep_odr'],
                convertSet: async (entity, key, value, meta) => {
                    await entity.write('customConfig', {[key]: sleep_odr[value]});
                    return {state: {[key]: value}};
                },
                convertGet: async (entity, key, meta) => {
                    await entity.read('customConfig', [key]);
                },
            }
            ,{
                key: ['active_odr'],
                convertSet: async (entity, key, value, meta) => {
                    await entity.write('customConfig', {[key]: active_odr[value]});
                    return {state: {[key]: value}};
                },
                convertGet: async (entity, key, meta) => {
                    await entity.read('customConfig', [key]);
                },
            }
        ];

        return {
            exposes,
            fromZigbee,
            toZigbee,
            isModernExtend: true,
        };
    },
    acceleration: () => {
        const flip_enums = ['pos', 'neg'];
        const exposes = [
            e.numeric('X', ea.STATE_GET)
                            .withUnit('g')
                            .withLabel('X')
                            .withCategory('diagnostic')
                            .withDescription('Acceleration on X axis'),
            e.numeric('Y', ea.STATE_GET)
                            .withUnit('g')
                            .withLabel('Y')
                            .withCategory('diagnostic')
                            .withDescription('Acceleration on Y axis'),
            e.numeric('Z', ea.STATE_GET)
                            .withUnit('g')
                            .withLabel('Z')
                            .withCategory('diagnostic')
                            .withDescription('Acceleration on Z axis'),
            e.text('WakeUp', ea.STATE_GET).withLabel('Last Wake Up').withCategory('diagnostic'),
            e.text('Sleep', ea.STATE_GET).withLabel('Last Sleep').withCategory('diagnostic'),
            e.text('Flip', ea.STATE_GET).withLabel('Last Flip').withCategory('diagnostic'),
            e.enum('flip_x', ea.STATE, flip_enums)
                .withCategory('diagnostic')
                .withDescription('Last Flip X event orientation')
                .withLabel('Last Flip X Orientation'),
            e.enum('flip_y', ea.STATE, flip_enums)
                .withCategory('diagnostic')
                .withDescription('Last Flip Y event orientation')
                .withLabel('Last Flip Y Orientation'),
            e.enum('flip_z', ea.STATE, flip_enums)
                .withCategory('diagnostic')
                .withDescription('Last Flip Z event orientation')
                .withLabel('Last Flip Z Orientation'),
        ];

        const fromZigbee = [
            {
                cluster: 'customAccel',
                type: ['attributeReport', 'readResponse'],
                convert: (model, msg, publish, options, meta) => {
                    const result = {};
                    const data = msg.data;
                    //logger.debug(`fZ convert attr: ${attr}; data:${util.inspect(data)}`, NS);
                    if ('X' in data) 
                        result['X'] = data['X']
                    if ('Y' in data) 
                        result['Y'] = data['Y']
                    if ('Z' in data) 
                        result['Z'] = data['Z']
                    if (Object.keys(result).length == 0) 
                        return;
                    return result;
                }
            },
            {
                cluster: 'customAccel',
                type: ['commandOn_wake_up'],
                convert: (model, msg, publish, options, meta) => {
                    var x = msg.data.x;
                    var y = msg.data.y;
                    var z = msg.data.z;
                    return {WakeUp: `${new Date().toLocaleString()}: x:${x}; y:${y}; z:${z};`};
                }
            },
            {
                cluster: 'customAccel',
                type: ['commandOn_sleep'],
                convert: (model, msg, publish, options, meta) => {
                    var x = msg.data.x;
                    var y = msg.data.y;
                    var z = msg.data.z;
                    return {Sleep: `${new Date().toLocaleString()}: x:${x}; y:${y}; z:${z};`};
                }
            },
            {
                cluster: 'customAccel',
                type: ['commandOn_flip'],
                convert: (model, msg, publish, options, meta) => {
                    var f = msg.data.flags;
                    var x = (f & (1 << 0)) != 0;
                    var y = (f & (1 << 1)) != 0;
                    var z = (f & (1 << 2)) != 0;

                    var x_neg = ((f >> 3) & 1);
                    var y_neg = ((f >> 4) & 1);
                    var z_neg = ((f >> 5) & 1);
                    return {Flip: `${new Date().toLocaleString()}: x:${x}; y:${y}; z:${z};`
                        , flip_x: flip_enums[x_neg]
                        , flip_y: flip_enums[y_neg]
                        , flip_z: flip_enums[z_neg]
                    };
                }
            }
        ];

        const toZigbee = [
            {
                key: ['X','Y','Z'],
                convertGet: async (entity, key, meta) => {
                    await entity.read('customAccel', [key]);
                },
            },
        ];

        return {
            exposes: exposes,
            fromZigbee,
            toZigbee,
            isModernExtend: true,
        };
    },
}

const definition = {
    zigbeeModel: ['Accel-NG'],
    model: 'Accel-NG',
    fingerprint: [{modelID: 'Accel-NG', applicationVersion: 1, priority: -1},],
    vendor: 'SFINAE',
    description: 'Overprogrammed Accelerometer Sensor',
    extend: [
        deviceAddCustomCluster('customAccel', {
            ID: 0xfc00,
            attributes: {
                X: {ID: 0x0000, type: Zcl.DataType.SINGLE_PREC},
                Y: {ID: 0x0001, type: Zcl.DataType.SINGLE_PREC},
                Z: {ID: 0x0002, type: Zcl.DataType.SINGLE_PREC},
            },
            commands: {
                on_wake_up: {
                    ID: 100,
                    parameters: [
                        {name: 'x', type: Zcl.DataType.SINGLE_PREC}
                        ,{name: 'y', type: Zcl.DataType.SINGLE_PREC}
                        ,{name: 'z', type: Zcl.DataType.SINGLE_PREC}
                    ],
                },
                on_sleep: {
                    ID: 101,
                    parameters: [
                        {name: 'x', type: Zcl.DataType.SINGLE_PREC}
                        ,{name: 'y', type: Zcl.DataType.SINGLE_PREC}
                        ,{name: 'z', type: Zcl.DataType.SINGLE_PREC}
                    ],
                },
                on_flip: {
                    ID: 102,
                    parameters: [{name: 'flags', type: Zcl.DataType.UINT8}]
                },
            },
            commandsResponse: {}
        }),
        deviceAddCustomCluster('customConfig', {
            ID: 0xfc01,
            attributes: {
                flags: {ID: 0x0000, type: Zcl.DataType.BITMAP32},
                wake_sleep_threshold: {ID: 0x0001, type: Zcl.DataType.UINT8},
                sleep_duration: {ID: 0x0002, type: Zcl.DataType.UINT8},
                sleep_odr: {ID: 0x0003, type: Zcl.DataType.ENUM8},
                active_odr: {ID: 0x0004, type: Zcl.DataType.ENUM8},
            },
            commands: {},
            commandsResponse: {}
        }),
        deviceAddCustomCluster('customStatus', {
            ID: 0xfc80,
            attributes: {
                status1: {ID: 0x0000, type: Zcl.DataType.INT16},
                status2: {ID: 0x0001, type: Zcl.DataType.INT16},
                status3: {ID: 0x0002, type: Zcl.DataType.INT16},
            },
            commands: {},
            commandsResponse: {}
        }),
        orlangurAccelExtended.acceleration(),
        orlangurAccelExtended.accelConfig(),
        orlangurAccelExtended.extendedStatus(),
        battery({
            voltage: true, 
            voltageReporting: true
        })
    ],
    configure: async (device, coordinatorEndpoint) => {
        const endpoint = device.getEndpoint(1);
        await reporting.bind(endpoint, coordinatorEndpoint, ['customAccel']);
        await reporting.bind(endpoint, coordinatorEndpoint, ['customStatus']);
        await endpoint.read('customAccel', ['X','Y','Z']);
        await endpoint.read('customConfig', [ 'flags', 'wake_sleep_threshold', 'sleep_duration', 'sleep_odr' ]);
        await endpoint.read('customStatus', [ 'status1', 'status2', 'status3']);
        await endpoint.configureReporting('customAccel', [
            {
                attribute: 'X',
                minimumReportInterval: 30,
                maximumReportInterval: constants.repInterval.HOUR,
                reportableChange: 0.1,
            },
        ]);
        await endpoint.configureReporting('customAccel', [
            {
                attribute: 'Y',
                minimumReportInterval: 30,
                maximumReportInterval: constants.repInterval.HOUR,
                reportableChange: 0.1,
            },
        ]);
        await endpoint.configureReporting('customAccel', [
            {
                attribute: 'Z',
                minimumReportInterval: 30,
                maximumReportInterval: constants.repInterval.HOUR,
                reportableChange: 0.1,
            },
        ]);

        await endpoint.configureReporting('customStatus', [
            {
                attribute: 'status1',
                minimumReportInterval: 30,
                maximumReportInterval: constants.repInterval.HOUR,
                reportableChange: 1,
            },
        ]);

        await endpoint.configureReporting('customStatus', [
            {
                attribute: 'status2',
                minimumReportInterval: 30,
                maximumReportInterval: constants.repInterval.HOUR,
                reportableChange: 1,
            },
        ]);

        await endpoint.configureReporting('customStatus', [
            {
                attribute: 'status3',
                minimumReportInterval: 30,
                maximumReportInterval: constants.repInterval.HOUR,
                reportableChange: 1,
            },
        ]);
    },

};

module.exports = definition;
