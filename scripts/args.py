import argparse
import re

PLATOON_PRESETS= {
    # scenario 1: 4 AVs with human cars inbetween, some of which are sensing cars used to collect metrics on
    'scenario1': 'human#sensor human*5 (human#sensor human*5 av human*5)*4 human#sensor human*5 human#sensor',
}

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--platoon', type=str, default='av human*5',
        help='Platoon of vehicles following the leader. Can contain either "human"s or "av"s. '
             '"(av human*2)*2" can be used as a shortcut for "av human human av human human". '
             'Vehicle tags can be passed with hashtags, eg "av#tag" "human#tag*3". '
             'Available presets: "scenario1".')
    parser.add_argument('--gui', default=False, action='store_true',
        help='Run in gui mode so that the platoon\'s progress can be watched during simulation. '
            'Has not been tested.')
    args = parser.parse_args()
    args.platoon = platoon_parse(args.platoon)
    return args

# From trajectory-training
def platoon_parse(platoon):
    if platoon in PLATOON_PRESETS:
        print(f'Setting scenario preset "{platoon}"')
        platoon = PLATOON_PRESETS[platoon]

    # replace (subplatoon)*n into subplatoon ... subplatoon (n times)
    replace1 = lambda match: ' '.join([match.group(1)] * int(match.group(2)))
    platoon = re.sub(r'\(([a-z0-9\s\*\#]+)\)\*([0-9]+)', replace1, platoon)
    # parse veh#tag1...#tagk*n into (veh, [tag1, ..., tagk], n)
    platoon_lst = re.findall(r'([a-z]+)((?:\#[a-z]+)*)(?:\*?([0-9]+))?', platoon)

    # spawn vehicles
    vehicles = []
    for vtype, vtags, vcount in platoon_lst:
        for _ in range(int(vcount) if vcount else 1):
            tags = vtags.split('#')[1:]
            if vtype == 'av':
                vehicles.append('av')
            elif vtype == 'human':
                vehicles.append('idm')
            else:
                raise ValueError(f'Unknown vehicle type: {vtype}. Allowed types are "human" and "av".')
    return vehicles
<<<<<<< HEAD
=======

>>>>>>> platoon-sim
