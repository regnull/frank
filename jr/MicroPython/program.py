import re

frank_commands = [
    "GO_IN",
    "GI",
    "FORWARD",
    "F",
    "HALF_FORWARD",
    "HF",
    "FORWARD_AND_CAPTURE",
    "FC",
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
    "ARC_LEFT",
    "AL",
    "ARC_RIGHT",
    "AR",
    "STOP"
]

def collapse_commands(cmd_list):
    if not cmd_list:
        return []

    collapsed = []
    current_cmd = cmd_list[0]
    count = 1

    for cmd in cmd_list[1:]:
        if cmd == current_cmd:
            count += 1
        else:
            # Only collapse FORWARD and BACKWARD commands
            if count > 1 and count <= 4 and current_cmd in ['FORWARD', 'BACKWARD']:
                collapsed.append(f"{count}{current_cmd}")
            else:
                # For other commands, append each instance separately
                for _ in range(count):
                    collapsed.append(current_cmd)
            current_cmd = cmd
            count = 1

    # Append the last accumulated command
    if count > 1 and count <= 4 and current_cmd in ['FORWARD', 'BACKWARD']:
        collapsed.append(f"{count}{current_cmd}")
    else:
        # For other commands, append each instance separately
        for _ in range(count):
            collapsed.append(current_cmd)

    return collapsed
  
# Normalize to short form for internal processing. Handles both short and long.
def _normalize(cmd):
    _TO_SHORT = {
        'GO_IN': 'GI', 'GI': 'GI',
        'FORWARD': 'F', 'F': 'F',
        'HALF_FORWARD': 'HF', 'HF': 'HF',
        'FORWARD_AND_CAPTURE': 'FC', 'FC': 'FC',
        'ADJUST': 'A', 'A': 'A',
        'BACKWARD': 'B', 'B': 'B',
        'TURN_LEFT': 'L', 'L': 'L',
        'TURN_RIGHT': 'R', 'R': 'R',
        'NORTH': 'N', 'N': 'N',
        'EAST': 'E', 'E': 'E',
        'SOUTH': 'S', 'S': 'S',
        'WEST': 'W', 'W': 'W',
        'ARC_LEFT': 'AL', 'AL': 'AL',
        'ARC_RIGHT': 'AR', 'AR': 'AR',
        'FORWARD_TO_TARGET': 'FT', 'FT': 'FT',
        'BACKWARD_TO_TARGET': 'BT', 'BT': 'BT',
        'DELAY': 'D', 'D': 'D',
        'STOP': 'STOP',
    }
    return _TO_SHORT.get(cmd, cmd)


def _orient_value(cmd):
    """Return 0=N, 1=E, 2=S, 3=W or None if not orientation."""
    if cmd in ('N', 'NORTH'): return 0
    if cmd in ('E', 'EAST'):  return 1
    if cmd in ('S', 'SOUTH'): return 2
    if cmd in ('W', 'WEST'):  return 3
    return None


def _is_turn(cmd):
    return cmd in ('L', 'R', 'TURN_LEFT', 'TURN_RIGHT',
                   'N', 'E', 'S', 'W', 'NORTH', 'EAST', 'SOUTH', 'WEST')


def _get_turn_type(orient, cmd):
    """
    Given current orientation (0–3: N,E,S,W) and a turn command,
    return 'L' (90 left), 'R' (90 right), 180 (error), 0 (no-op), or None (not a turn).
    """
    if cmd in ('L', 'TURN_LEFT'):
        return 'L'
    if cmd in ('R', 'TURN_RIGHT'):
        return 'R'
    ov = _orient_value(cmd)
    if ov is None:
        return None
    # absolute: delta in 90° steps
    delta = (ov - orient + 4) % 4
    if delta == 0: return 0
    if delta == 1: return 'R'
    if delta == 2: return 180
    if delta == 3: return 'L'
    return None


def _apply_turn(orient, turn):
    """Apply AL/AR or L/R to orientation. turn in ('L','R','AL','AR')."""
    if turn in ('L', 'AL'):
        return (orient - 1 + 4) % 4
    if turn in ('R', 'AR'):
        return (orient + 1) % 4
    return orient


def translate_for_capture(cmd_list):
    """
    Translate capture sequences FC...B in three passes:
    - First:  F -> HF,HF  and  FC -> HF,HF
    - Second: [HF, 90° turn, HF] -> [AL] or [AR] (AL/AR = forward half + turn); else [HF, turn] -> [HF, AL/AR]
    - Third:  consecutive HF,HF -> F
    """
    if not cmd_list:
        return []

    # Normalize to short form
    cmds = [_normalize(c) for c in cmd_list]

    out = []
    i = 0
    orient = None  # 0=N,1=E,2=S,3=W; None until set by N/E/S/W

    while i < len(cmds):
        c = cmds[i]

        # Set or update orientation for absolute compass
        ov = _orient_value(c)
        if ov is not None:
            orient = ov
            out.append(c)
            i += 1
            continue

        # Relative turns (L/R) when we don't have orient yet: keep as-is, assume 90
        if c in ('L', 'R'):
            if orient is not None:
                orient = _apply_turn(orient, c)
            out.append(c)
            i += 1
            continue

        # FC starts a capture sequence
        if c != 'FC':
            # Pass through (F, B, A, HF, AL, AR, GI, FT, BT, D, STOP, etc.)
            if c in ('AL', 'AR') and orient is not None:
                orient = _apply_turn(orient, c)
            out.append(c)
            i += 1
            continue

        # Find next B for sequence [FC, ..., B]
        j = i + 1
        while j < len(cmds) and cmds[j] != 'B':
            j += 1
        if j >= len(cmds):
            raise ValueError("Capture sequence starting with FC has no matching B")

        seq = cmds[i : j + 1]  # FC and up to and including B

        # Assume N if no orientation set yet (e.g. FC at start of program)
        o = 0 if orient is None else orient
        orient_at_seq_start = o

        # 1) Validate: no A, no 180° turn (including L,L or R,R)
        for k, cmd in enumerate(seq):
            if cmd in ('A', 'ADJUST'):
                raise ValueError("ADJUST is not allowed in a capture sequence (bottle obscures sensor)")
            t = _get_turn_type(o, cmd) if _is_turn(cmd) else None
            if t == 180:
                raise ValueError("180-degree turn is not allowed in a capture sequence")
            if t in ('L', 'R'):
                o = _apply_turn(o, cmd if cmd in ('L', 'R') else ('L' if t == 'L' else 'R'))
                # Detect 180 from sequence start (e.g. L,L or R,R)
                if (o - orient_at_seq_start + 4) % 4 == 2:
                    raise ValueError("180-degree turn is not allowed in a capture sequence")
            elif t == 0 and _orient_value(cmd) is not None:
                pass  # no-op orientation
            elif cmd in ('AL', 'AR'):
                o = _apply_turn(o, cmd)
                if (o - orient_at_seq_start + 4) % 4 == 2:
                    raise ValueError("180-degree turn is not allowed in a capture sequence")

        # First pass: F -> HF,HF and FC -> HF,HF
        pre = []
        for cmd in seq:
            if cmd == 'FC':
                pre.extend(['HF', 'HF'])
            elif cmd == 'F':
                pre.extend(['HF', 'HF'])
            else:
                pre.append(cmd)

        # Second pass: [HF, 90° turn, HF] -> [AL] or [AR]. AL/AR = forward half + turn.
        # Optional: [HF, 90° turn] with no HF after -> [HF, AL/AR].
        if orient is None:
            orient = 0
        trans = []
        skip_next_hf = False

        for k, cmd in enumerate(pre):
            if skip_next_hf and cmd == 'HF':
                skip_next_hf = False
                continue
            if cmd == 'HF':
                trans.append('HF')
                continue
            if cmd == 'B':
                trans.append('B')
                continue
            if cmd in ('AL', 'AR'):
                if orient is not None:
                    orient = _apply_turn(orient, cmd)
                trans.append(cmd)
                continue

            # Turn: L, R, or N,E,S,W
            t = _get_turn_type(orient, cmd) if _is_turn(cmd) else None
            if t is None:
                trans.append(cmd)
                continue
            if t == 180:
                raise ValueError("180-degree turn is not allowed in a capture sequence")
            if t == 0:
                continue
            nxt = pre[k + 1] if k + 1 < len(pre) else None
            if nxt == 'HF':
                # [HF, turn, HF] -> [AL]: pop the run-up HF, append AL/AR, skip the HF after
                if trans and trans[-1] == 'HF':
                    trans.pop()
                if t == 'L':
                    trans.append('AL')
                else:
                    trans.append('AR')
                skip_next_hf = True
            else:
                # [HF, turn] with no HF after -> [HF, AL/AR]; keep the run-up HF, append AL/AR
                if t == 'L':
                    trans.append('AL')
                else:
                    trans.append('AR')
            if orient is not None:
                orient = _apply_turn(orient, 'L' if t == 'L' else 'R')

        # Third pass: consecutive HF, HF -> F
        glued = []
        idx = 0
        while idx < len(trans):
            if idx + 1 < len(trans) and trans[idx] == 'HF' and trans[idx + 1] == 'HF':
                glued.append('F')
                idx += 2
            else:
                glued.append(trans[idx])
                idx += 1

        out.extend(glued)
        i = j + 1

    return out

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
    commands = [self._translate_command(cmd) for cmd in commands]
    self.commands = collapse_commands(commands)
    
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
  
  def remaining_commands(self):
    return self.commands[self.current_command:]
  
  def _translate_command(self, cmd):
    '''Always translate commands to the full command name'''
    
    if cmd == 'GI':
      return 'GO_IN'
    elif cmd == 'F':
      return 'FORWARD'
    elif cmd == 'HF':
      return 'HALF_FORWARD'
    elif cmd == 'FC':
      return 'FORWARD_AND_CAPTURE'
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
    elif cmd == 'AL':
      return 'ARC_LEFT'
    elif cmd == 'AR':
      return 'ARC_RIGHT'
    return cmd

def main():
  program = Program()
  program.load('/Users/regnull/work/frank/jr/MicroPython/program.frank')
  print(program.commands)

if __name__ == "__main__":
  main()