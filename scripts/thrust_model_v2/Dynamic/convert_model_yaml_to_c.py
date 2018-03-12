import sys
import datetime
import os
import yaml

# Process a settings file and output 
# a C style array for use in a C++
# environment without yaml loading capabilities

if __name__ == "__main__":
    filename = sys.argv[1]

    with open(filename) as f:
        settings = yaml.load(f)

    model_name = os.path.split(filename)[1].split('.txt')[0]

    with open('TestThrustModel/' + model_name+'.h', 'w') as f:

        header_guard = 'PROP_' + model_name

        f.write('// Thrust model for {}\n'.format(model_name))
        f.write('// Generated on {}\n\n'.format(str(datetime.datetime.now())))
        f.write('#ifndef {}_H\n'.format(header_guard))
        f.write('#define {}_H\n\n'.format(header_guard))
        f.write('const float response_lag = {};\n'.format(settings['response_lag']))
        f.write('const float thrust_min = {};\n'.format(settings['voltage_to_jerk']['thrust_min']))
        f.write('const float thrust_max = {};\n'.format(settings['voltage_to_jerk']['thrust_max']))
        f.write('const float voltage_min = {};\n'.format(settings['voltage_to_jerk']['voltage_min']))
        f.write('const float voltage_max = {};\n'.format(settings['voltage_to_jerk']['voltage_max']))
        f.write('const float timestep = {};\n'.format(settings['voltage_to_jerk']['timestep']))
        f.write('\n')

        thrust_to_voltage_order = len(settings['thrust_to_voltage'])-1
        f.write('const uint8_t thrust_to_voltage_order = {};\n'.format(thrust_to_voltage_order))
        f.write('const float thrust_to_voltage[{}] = {{{}}};'.format(thrust_to_voltage_order+1,
                                                          str(settings['thrust_to_voltage'])[1:-1-1]))
        f.write('\n\n')

        model = settings['voltage_to_jerk']['mapping']
        num_thrust_points = len(model)
        num_voltage_points = len(model[0][1])
        f.write('const uint8_t num_thrust_points = {};\n'.format(num_thrust_points))
        f.write('const uint8_t num_voltage_points = {};\n'.format(num_voltage_points))

        f.write('const float voltage_to_jerk_mapping[{}][{}][2] = {{\n'.format(num_thrust_points, num_voltage_points+1))

        for i in range(0, num_thrust_points):
            line = '    {{{{{}, 0}}'.format(model[i][0])
            for j in range(0, num_voltage_points):
                line = line + ', {{{}, {}}}'.format(model[i][1][j][0], model[i][1][j][1])
            line = line + '}'
            if i < num_thrust_points - 1:
                line = line + ','
            f.write(line)
            f.write('\n')
        f.write('};')

        f.write('\n\n')
        f.write('#endif\n')
