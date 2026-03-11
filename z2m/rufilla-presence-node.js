const {Zcl} = require('zigbee-herdsman');

const manufacturerCode = 0x1234;

const definition = {
    zigbeeModel: ['presence-node-v1'],
    model: 'presence-node-v1',
    vendor: 'Rufilla',
    description: 'Rufilla Intelligence Node — mmWave presence + ToF ranging',
    fromZigbee: [
        {
            cluster: 'msOccupancySensing',
            type: ['attributeReport', 'readResponse'],
            convert: (model, msg, publish, options, meta) => {
                if (msg.data.occupancy !== undefined) {
                    return {occupancy: msg.data.occupancy > 0};
                }
            },
        },
        {
            cluster: 0xFC00,
            type: ['attributeReport', 'readResponse'],
            convert: (model, msg, publish, options, meta) => {
                const result = {};
                if (msg.data['1'] !== undefined) result.moving_target = msg.data['1'] > 0;
                if (msg.data['2'] !== undefined) result.stationary_target = msg.data['2'] > 0;
                if (msg.data['3'] !== undefined) result.move_energy = msg.data['3'];
                if (msg.data['4'] !== undefined) result.static_energy = msg.data['4'];
                if (msg.data['5'] !== undefined) result.target_distance_cm = msg.data['5'];
                return result;
            },
        },
        {
            cluster: 0xFC01,
            type: ['attributeReport', 'readResponse'],
            convert: (model, msg, publish, options, meta) => {
                const result = {};
                if (msg.data['1'] !== undefined) result.range_mm = msg.data['1'];
                if (msg.data['2'] !== undefined) result.range_status = msg.data['2'];
                return result;
            },
        },
    ],
    toZigbee: [],
    exposes: [
        {
            type: 'binary',
            name: 'occupancy',
            property: 'occupancy',
            value_on: true,
            value_off: false,
            access: 1, // STATE_GET
            description: 'Occupancy detected (mmWave)',
        },
        {
            type: 'binary',
            name: 'moving_target',
            property: 'moving_target',
            value_on: true,
            value_off: false,
            access: 1,
            description: 'Moving target detected',
        },
        {
            type: 'binary',
            name: 'stationary_target',
            property: 'stationary_target',
            value_on: true,
            value_off: false,
            access: 1,
            description: 'Stationary target detected',
        },
        {
            type: 'numeric',
            name: 'move_energy',
            property: 'move_energy',
            value_min: 0,
            value_max: 100,
            access: 1,
            description: 'Moving target energy level',
        },
        {
            type: 'numeric',
            name: 'static_energy',
            property: 'static_energy',
            value_min: 0,
            value_max: 100,
            access: 1,
            description: 'Stationary target energy level',
        },
        {
            type: 'numeric',
            name: 'target_distance_cm',
            property: 'target_distance_cm',
            unit: 'cm',
            access: 1,
            description: 'Target distance (mmWave)',
        },
        {
            type: 'numeric',
            name: 'range_mm',
            property: 'range_mm',
            unit: 'mm',
            access: 1,
            description: 'VL53L0X measured range',
        },
        {
            type: 'numeric',
            name: 'range_status',
            property: 'range_status',
            value_min: 0,
            value_max: 255,
            access: 1,
            description: 'VL53L0X range status (0=valid, 1=sigma fail, 2=signal fail, 255=no target)',
        },
    ],
    configure: async (device, coordinatorEndpoint, logger) => {
        const endpoint = device.getEndpoint(1);

        // Bind standard occupancy cluster
        await endpoint.bind('msOccupancySensing', coordinatorEndpoint);

        // Configure reporting for occupancy (5 second interval)
        await endpoint.configureReporting('msOccupancySensing', [
            {attribute: 'occupancy', minimumReportInterval: 5, maximumReportInterval: 5, reportableChange: 0},
        ]);

        // Read initial values from custom clusters
        try {
            await endpoint.read(0xFC00, [1, 2, 3, 4, 5], {manufacturerCode});
            await endpoint.read(0xFC01, [1, 2], {manufacturerCode});
        } catch (e) {
            // Custom cluster reads may fail before first report — non-fatal
        }
    },
};

module.exports = definition;
