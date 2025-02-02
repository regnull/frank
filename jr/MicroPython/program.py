import re

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

class Program:
  def __init__(self):
    self.time_goal = 0
    self.commands = []
    self.current_command = 0
      
  def load(self, file_name):
    self.current_command = 0
    self.commands = []
    
    time_goal = None
    commands = []

    with open(file_name, 'r') as file:
      for line in file:
        line = line.strip()

        # 1. Ignore comment lines and blank lines
        if not line or line.startswith('#'):
            continue

        # If we haven't yet found the TIME_GOAL, the next non-comment, non-blank line
        # must match something like: TIME_GOAL   =   60
        if time_goal is None:
          # Simple pattern allowing optional spaces around '=' and capturing digits
          match = re.match('^TIME_GOAL[ \t]*=[ \t]*([0-9]+)[ \t]*$', line)
          if match:
              # Extract numeric value
              try:
                  time_goal = int(match.group(1))
              except ValueError:
                  raise ValueError("TIME_GOAL must be an integer: {}".format(line))
          else:
              raise ValueError("TIME_GOAL must appear before any commands, "
                              "in the form 'TIME_GOAL = <number>'.")
        else:
          split_cmds = line.split(',')
          for cmd in split_cmds:
            cmd = cmd.strip()
            if cmd:
              commands.append(cmd)

    # Final checks
    if time_goal is None:
        raise ValueError("Missing TIME_GOAL line in the input file.")

    if not commands:
        raise ValueError("No commands found after TIME_GOAL.")
      
    if commands[-1] != "STOP":
      commands.append("STOP")

    self.time_goal = float(time_goal)
    self.commands = commands
    for command in self.commands:
      if command not in commands:
        raise ValueError("Unknown command: ", command)

  def reset(self):
    self.current_command = 0

  def next_command(self):
    if self.current_command >= len(self.commands):
      return None
    
    cmd = self.commands[self.current_command]
    self.current_command += 1
    return self._translate_command(cmd)

  def has_more_commands(self):
    return self.current_command < len(self.commands)
  
  def _translate_command(self, cmd):
    '''Always translate commands to the full command name'''
    
    if cmd == 'GI':
      return 'GO_IN'
    elif cmd == 'F':
      return 'FORWARD'
    elif cmd == 'A':
      return 'ADJUST'
    elif cmd == 'B':
      return 'BACKWARD'
    elif cmd == 'L':
      return 'TURN_LEFT'
    elif cmd == 'R':
      return 'TURN_RIGHT'
    elif cmd == 'FT':
      return 'FORWARD_TO_TARGET'
    elif cmd == 'BT':
      return 'BACKWARD_TO_TARGET'
    elif cmd == 'D':
      return 'DELAY'
    elif cmd == 'N':
      return 'NORTH'
    elif cmd == 'E':
      return 'EAST'
    elif cmd == 'S':
      return 'SOUTH'
    elif cmd == 'W':
      return 'WEST'
    elif cmd == 'STOP':
      return 'STOP'
    return cmd

def main():
  program = Program()
  program.load('/Users/regnull/work/frank/jr/MicroPython/program.frank')
  print(program.commands)

if __name__ == "__main__":
  main()