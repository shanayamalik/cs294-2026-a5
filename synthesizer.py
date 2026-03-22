# sample use:
# python3 synthesizer.py 4 4 0 0 2 0 ""
# python3 synthesizer.py 5 5 0 0 4 0 "2,0;2,1;2,2"
# see the section below on parsing obstacles for more details

from z3 import *
import sys

class Synthesizer(object):

    # --------------------------------------------------------
    # Environment and task details
    # --------------------------------------------------------

    def __init__(self, width, height, start_x_coord, start_y_coord, goal_x_coord, goal_y_coord, obstacles):
        self.width = width
        self.height = height
        self.start_x_coord = start_x_coord
        self.start_y_coord = start_y_coord
        self.goal_x_coord = goal_x_coord
        self.goal_y_coord = goal_y_coord
        self.obstacles = obstacles

    # --------------------------------------------------------
    # The fun stuff :)
    # --------------------------------------------------------

    # what's the effect of running one instruction?  where does the robot end up?
    # 0=L, 1=R, 2=D, 3=U. Unit moves, clamped to grid.
    def run_instr(self, x, y, instr):
        new_x = If(instr == 0,
                    If(x - 1 >= 0, x - 1, x),
                 If(instr == 1,
                    If(x + 1 <= self.width - 1, x + 1, x),
                 x))
        new_y = If(instr == 2,
                    If(y + 1 <= self.height - 1, y + 1, y),
                 If(instr == 3,
                    If(y - 1 >= 0, y - 1, y),
                 y))
        return new_x, new_y

    # what's the effect of running the whole program?  where does the robot end up?
    def run_prog(self, x, y, instrs):
        curr_x, curr_y = x, y
        for i in range(len(instrs)):
            curr_x, curr_y = self.run_instr(curr_x, curr_y, instrs[i])
        return curr_x, curr_y

    # let's make some Z3 bitvectors that we'll use to search the space of instructions
    def gen_instrs(self, num_instrs):
        return [BitVec('instr_'+str(i), 2) for i in range(num_instrs)]

    # extract instruction sequence from Z3 model
    def instrs_in_seq(self, model, instrs):
        return [model.eval(instrs[i]) for i in range(len(instrs))]

    # a convenience function for printing the output to look like a sequence of instructions
    def print_instrs(self, instrs):
        names = {0: "L", 1: "R", 2: "D", 3: "U"}
        for val in instrs:
            v = val.as_long() if hasattr(val, 'as_long') else int(str(val))
            print(names.get(v, "-"))

    def synthesize(self):
        num_instrs = 4  # example value. shouldn't have hard-coded num_instrs in HW!
        instrs = self.gen_instrs(num_instrs)  # generate BVs to represent instructions
        final_x, final_y = self.run_prog(self.start_x_coord, self.start_y_coord, instrs)
        goal = And(final_x == self.goal_x_coord, final_y == self.goal_y_coord)  # where do we want our robot to move?

        s = Solver()
        s.add(goal)
        satisfiable = s.check()

        if satisfiable == sat:
            model = s.model()
            instrs = self.instrs_in_seq(model, instrs)
            self.print_instrs(instrs)  # print the program if we found one
            return instrs

def parse_obstacles(s):
    if not s or s.strip() == "":
        return []
    coords = s.split(";")
    return [(int(comps[0]), int(comps[1])) for comps in [coord.split(",") for coord in coords]]

if __name__ == '__main__':
    # --------------------------------------------------------
    # Parse command line args
    # --------------------------------------------------------

    grid_width = int(sys.argv[1])   # how many squares across?
    grid_height = int(sys.argv[2])  # how many squares up and down?
    start_x_coord = int(sys.argv[3])  # with leftmost column at coord 0, coord of start square
    start_y_coord = int(sys.argv[4])  # with topmost row at coord 0, coord of start square
    goal_x_coord = int(sys.argv[5])   # with leftmost column at coord 0, coord of goal square
    goal_y_coord = int(sys.argv[6])   # with topmost row at coord 0, coord of goal square

    obstacles = []
    if len(sys.argv) >= 8:
        obstacles = parse_obstacles(sys.argv[7])

    synth = Synthesizer(grid_width, grid_height, start_x_coord, start_y_coord, goal_x_coord, goal_y_coord, obstacles)
    synth.synthesize()

