import re

frank_commands = [
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
      line_number = 0
      for line in file:
        line_number += 1
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
              time_goal_str = match.group(1)
              # Extract numeric value
              try:
                  time_goal = int(time_goal_str)
              except ValueError:
                  raise ValueError(f"bad time goal: {time_goal_str}, line: {line_number}")
              if time_goal < 20:
                time_goal = 20
              if time_goal > 240:
                time_goal = 240
          else:
              raise ValueError("TIME_GOAL must appear before any commands, "
                              "in the form 'TIME_GOAL = <number>'.")
        else:
          split_cmds = line.split(',')
          for cmd in split_cmds:
            cmd = cmd.strip()
            if cmd:
              if cmd not in frank_commands:
                raise ValueError(f"Bad cmd: {cmd} at line: {line_number}")
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