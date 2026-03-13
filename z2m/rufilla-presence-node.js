const definition = {
    zigbeeModel: ['presence-node-v1'],
    model: 'presence-node-v1',
    vendor: 'Rufilla',
    description: 'Rufilla Intelligence Node — mmWave presence sensor',
    fromZigbee: [
        {
            cluster: 'genAnalogInput',
            type: ['attributeReport', 'readResponse'],
            convert: (model, msg, publish, options, meta) => {
                if (msg.data.presentValue !== undefined) {
                    return {presence: msg.data.presentValue > 0.5};
                }
            },
        },
    ],
    toZigbee: [],
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
    ],
    configure: async (device, coordinatorEndpoint, logger) => {
        const endpoint = device.getEndpoint(1);
        if (endpoint) {
            await endpoint.bind('genAnalogInput', coordinatorEndpoint);
        }
    },
};

module.exports = definition;
