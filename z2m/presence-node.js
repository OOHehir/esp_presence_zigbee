const definition = {
    zigbeeModel: ['presence-node-v1'],
    model: 'presence-node-v1',
    vendor: 'ElectronicsConsult',
    description: 'ElectronicsConsult Presence Node — mmWave presence + ToF ranging',
    fromZigbee: [
        {
            cluster: 'genAnalogInput',
            type: ['attributeReport', 'readResponse'],
            convert: (model, msg, publish, options, meta) => {
                if (msg.data.presentValue === undefined) return;
                const val = msg.data.presentValue;

                switch (msg.endpoint.ID) {
                case 1: return {presence: val > 0.5};
                case 2: return {range_cm: Math.round(val)};
                case 3: return {range_status: Math.round(val)};
                }
            },
        },
    ],
    toZigbee: [],
    meta: {multiEndpoint: true},
    endpoint: (device) => ({
        presence: 1,
        range_cm: 2,
        range_status: 3,
    }),
    exposes: [
        {
            type: 'binary',
            name: 'presence',
            label: 'Presence',
            property: 'presence',
            value_on: true,
            value_off: false,
            access: 1,
            description: 'Presence detected (mmWave)',
        },
        {
            type: 'numeric',
            name: 'range_cm',
            label: 'Range',
            property: 'range_cm',
            unit: 'cm',
            access: 1,
            description: 'VL53L0X measured range',
        },
        {
            type: 'numeric',
            name: 'range_status',
            label: 'Range status',
            property: 'range_status',
            access: 1,
            description: 'VL53L0X measurement status (0=valid, 255=no target)',
        },
    ],
    configure: async (device, coordinatorEndpoint, logger) => {
        for (const ep of [1, 2, 3]) {
            const endpoint = device.getEndpoint(ep);
            if (endpoint) {
                await endpoint.bind('genAnalogInput', coordinatorEndpoint);
            }
        }
    },
};

module.exports = definition;
