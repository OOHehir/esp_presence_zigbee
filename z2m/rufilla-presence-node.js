const fz = {
    analog_input: {
        cluster: 'genAnalogInput',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const val = msg.data.presentValue;
            if (val === undefined) return;

            switch (msg.endpoint.ID) {
            case 1: return {presence: val > 0.5};
            case 2: return {range_cm: Math.round(val)};
            }
        },
    },
};

const definition = {
    zigbeeModel: ['presence-node-v1'],
    model: 'presence-node-v1',
    vendor: 'Rufilla',
    description: 'Rufilla Intelligence Node — mmWave presence + ToF ranging',
    fromZigbee: [fz.analog_input],
    toZigbee: [],
    meta: {multiEndpoint: true},
    endpoint: (device) => ({
        presence: 1,
        range_cm: 2,
    }),
    exposes: [
        {
            type: 'binary',
            name: 'presence',
            property: 'presence',
            value_on: true,
            value_off: false,
            access: 1,
            description: 'Presence detected (mmWave)',
        },
        {
            type: 'numeric',
            name: 'range_cm',
            property: 'range_cm',
            unit: 'cm',
            access: 1,
            description: 'VL53L0X measured range',
        },
    ],
    configure: async (device, coordinatorEndpoint, logger) => {
        for (const ep of [1, 2]) {
            const endpoint = device.getEndpoint(ep);
            if (endpoint) {
                await endpoint.bind('genAnalogInput', coordinatorEndpoint);
            }
        }
    },
};

module.exports = definition;
