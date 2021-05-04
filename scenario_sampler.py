import os
import argparse
import json
import random
import copy
import datetime

def main(route, num_run):
    # print(route)
    # print(num_run)
    now = str(datetime.datetime.now())
    now = now.replace(' ','_')
    now = now.replace(':','-')
    now = now.split('.')[0]
    log_dir = './log/log_'+now
    os.makedirs(log_dir)

    scenario_config_filename = route[1]
    scenario_config = json.load(open(scenario_config_filename,'r'))
    agent_path = scenario_config['ego_vehicle']['controller']
    
    for j in range(num_run):
        sampled_config = copy.deepcopy(scenario_config)
        
        # Sample initial condition for ego vehicle
        for key in sampled_config['ego_vehicle']['transform']:
            val_range = sampled_config['ego_vehicle']['transform'][key]
            if type(val_range) == list and len(val_range) > 1:
                val = random.uniform(val_range[0], val_range[1])
                print(f'{key}: {val}')
                sampled_config['ego_vehicle']['transform'][key] = val
            elif type(val_range) == list:
                sampled_config['ego_vehicle']['transform'][key] = val_range[0]    
        
        sampled_config['available_scenarios'][0]['Town05'][0]['available_event_configurations'][0]['transform'] = sampled_config['ego_vehicle']['transform']

        # Sample initial condition for NPC vehicle
        for i in range(len(sampled_config['npc_vehicle'])):
            npc_vehicle = sampled_config['npc_vehicle'][i]
            for key in npc_vehicle['relative_transform']:
                val_range = npc_vehicle['relative_transform'][key]
                if type(val_range) == list and len(val_range) > 1:
                    val = random.uniform(val_range[0], val_range[1])
                    print(f'{key}: {val}')
                    sampled_config['npc_vehicle'][i]['relative_transform'][key] = val
                elif type(val_range) == list:
                    sampled_config['npc_vehicle'][i]['relative_transform'][key] = val_range[0]
                    
        # Sample parameter for each behavior
        for i in range(len(sampled_config['behavior'])):
            action = sampled_config['behavior'][i]
            for key in action['params']:
                val_range = action['params'][key]
                if type(val_range) == list and len(val_range) > 1:
                    val = random.uniform(val_range[0], val_range[1])
                    print(f'{key}: {val}')
                    sampled_config['behavior'][i]['params'][key] = val
                elif type(val_range) == list:
                    sampled_config['behavior'][i]['params'][key] = val_range[0]
                

        print(sampled_config)
        sampled_config_fn = './sampled_config.json'
        json.dump(sampled_config,open(sampled_config_fn,'w+'))

        stored_config_fn = f'{log_dir}/test_run_{j}.json'
        json.dump(sampled_config,open(stored_config_fn,'w+'))

        route_config = f"{route[0]} {sampled_config_fn} 0"
        cmd = f'python3 scenario_runner.py --route {route_config} --agent {agent_path} --output \
            --run_idx {j} --log_dir {log_dir}'
        os.system(cmd)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = "test framework")

    parser.add_argument('--route', help='Run a route as a scenario', nargs='+', type=str)
    parser.add_argument('--num_run', help='Number of test runs', type=int)
    arguments = parser.parse_args()

    main(arguments.route, arguments.num_run)
