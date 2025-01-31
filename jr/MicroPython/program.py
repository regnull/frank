import json

commands = [
    "GO_IN",
    "GI",
    "FORWARD",
    "F",
    "ADJUST",
    "A",
    "BACKWARD",
    "B",
    "TURN_LEFT",
    "L",
    "TURN_RIGHT",
    "R",
    "LEFT_SHIFT",
    "LS",
    "RIGHT_SHIFT",
    "RS",
    "TEST_MOVE",
    "FORWARD_TO_TARGET",
    "FT",
    "BACKWARD_TO_TARGET",
    "BT",
    "DELAY",
    "D",
    "NORTH",
    "N",
    "EAST",
    "E",
    "SOUTH",
    "S",
    "WEST",
    "W",
    "STOP"
]

def parse_program(program):
  with open(program, 'r') as f:
    p = json.load(f)
    time_goal = p['time_goal']
    commands = []
    for command in p['commands']:
      c = command.split(',')
      for cmd in c:
        commands.append(cmd.strip())

    for command in commands:
      if command not in commands:
        raise ValueError("Unknown command: ", command)

    return time_goal, commands

'''
if(s == "GO_IN" || s == "GI") {
    return GO_IN;
  }
  if(s == "FORWARD" || s == "F") {
    return FORWARD;
  }
  if(s == "ADJUST" || s == "A") {
    return ADJUST;
  }
  if(s == "BACKWARD" || s == "B") {
    return BACKWARD;
  } 
  if(s == "TURN_LEFT" || s == "L") {
    return TURN_LEFT;
  }
  if(s == "TURN_RIGHT" || s == "R") {
    return TURN_RIGHT;
  }
  if(s == "LEFT_SHIFT" || s == "LS") {
    return LEFT_SHIFT;
  }
  if(s == "RIGHT_SHIFT" || s == "RS") {
    return RIGHT_SHIFT;
  }
  if(s == "TEST_MOVE") {
    return TEST_MOVE;
  }
  if(s == "FORWARD_TO_TARGET" || s == "FT") {
    return FORWARD_TO_TARGET;
  }
  if(s == "BACKWARD_TO_TARGET" || s == "BT") {
    return BACKWARD_TO_TARGET;
  }
  if(s == "DELAY" || s == "D") {
    return DELAY;
  }
  if(s == "NORTH" || s == "N") {
    return NORTH;
  }
  if(s == "EAST" || s == "E") {
    return EAST;
  }
  if(s == "SOUTH" || s == "S") {
    return SOUTH;
  }
  if(s == "WEST" || s == "W") {
    return WEST;
  }
  if( s == "STOP") {
    return STOP;
  }
'''

