const { Buffer } = require('node:buffer');
const util = require('node:util');
const {Zcl} = require('zigbee-herdsman');
const {enumLookup,numeric,deviceAddCustomCluster,onOff,binary} = require('zigbee-herdsman-converters/lib/modernExtend');
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
    accelConfig: () => {
        const exposes = [
            e.binary('activity_x', ea.ALL, 1, 0).withCategory('config').withDescription('Sensing activity on X'),
            e.binary('activity_y', ea.ALL, 1, 0).withCategory('config').withDescription('Sensing activity on Y'),
            e.binary('activity_z', ea.ALL, 1, 0).withCategory('config').withDescription('Sensing activity on Z'),
        ];

        const cfg_bits = {
            activity_x  : 0,
            activity_y  : 1,
            activity_z  : 2,
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
                        set_cfg('activity_x', b0);
                        set_cfg('activity_y', b0);
                        set_cfg('activity_z', b0);
                    }

                    if (Object.keys(result).length == 0) 
                        return;

                    return result;
                }
            }
        ];

        const toZigbee = [{
            key: ['activity_x', 'activity_y', 'activity_z'],
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
        }];

        return {
            exposes,
            fromZigbee,
            toZigbee,
            isModernExtend: true,
        };
    },
    acceleration: () => {
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
            e.numeric('LastEvent', ea.STATE_GET)
                            .withLabel('LastEvent')
                            .withCategory('diagnostic'),
            e.numeric('EventCount', ea.STATE_GET)
                            .withLabel('Event Count')
                            .withCategory('diagnostic'),
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
                type: ['commandNotification'],
                convert: (model, msg, publish, options, meta) => {
                    const commandID = msg.data.commandID
                    if (commandID == 100)
                    {
                        const payloadBuf = Buffer.from(msg.data.commandFrame.raw);
                        const param1 = payloadBuf.readUInt32LE(0);
                        const cnt = meta.state.EventCount;
                        return {LastEvent: param1, EventCount: cnt + 1};
                    }
                    return;
                }
            }
        ];

        const toZigbee = [
            {
                key: ['X','Y','Z'],
                convertGet: async (entity, key, meta) => {
                    await entity.read('customAccel', [key]);
                },
            }
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
                on_event: {
                    ID: 100,
                    parameters: [{name: 'flags', type: Zcl.DataType.UINT32}],
                },
            },
            commandsResponse: {}
        }),
        deviceAddCustomCluster('customConfig', {
            ID: 0xfc01,
            attributes: {
                flags: {ID: 0x0000, type: Zcl.DataType.BITMAP32},
            },
            commands: {},
            commandsResponse: {}
        }),
        orlangurAccelExtended.acceleration(),
        orlangurAccelExtended.accelConfig(),
    ],
    configure: async (device, coordinatorEndpoint) => {
        const endpoint = device.getEndpoint(1);
        await reporting.bind(endpoint, coordinatorEndpoint, ['customAccel']);
        await endpoint.read('customAccel', ['X','Y','Z']);
        await endpoint.read('customConfig', [ 'flags' ]);
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
    },

};

module.exports = definition;
