# sample use: 
# python3 synthesizer.py 16 5 7 0 12 4 "2,2;1,1"
# python3 synthesizer.py 5 5 0 0 4 0 "2,0;2,1;2,2"
# see the section below on parsing obstacles for more details

from z3 import *
import sys

class Synthesizer(object):

    # --------------------------------------------------------
    # Environment and task details
    # Useful for HW; you may want to use more of the inputs!
    # --------------------------------------------------------

    def __init__(self, width, height, start_x_coord, start_y_coord, goal_x_coord, goal_y_coord, obstacles):
        self.width = width
        self.height = height

        self.start_x_coord = start_x_coord
        self.goal_x_coord = goal_x_coord

    # --------------------------------------------------------
    # The fun stuff :)
    # --------------------------------------------------------

    # what's the effect of running one instruction?  where does the robot end up?
    def run_instr(self, pos, instr, arg, envir):
        return If(arg > 0,
                If(instr == 0,
                    # left
                    If(pos - arg >= 0, pos - arg, 0),
                If(instr == 1,
                    # right
                    If(pos + arg <= (envir - 1), pos + arg, envir - 1),
                pos)),
                pos)

    # what's the effect of running the whole program?  where does the robot end up?
    def run_prog(self, pos, instrs, args, envir):
        curr_pos = pos
        for i in range(len(instrs)):
            curr_pos = self.run_instr(curr_pos, instrs[i], args[i], envir)
        return curr_pos

    # let's make some Z3 bitvectors that we'll use to search the space of instructions
    def gen_instrs(self, num_instrs):
        return [BitVec('x_'+str(i),2) for i in range(num_instrs)] # try bitwidth 1 and 2 with num_instrs 6


    # let's make some Z3 ints that we'll use to search the space of arguments
    def gen_args(self, num_instrs):
        return [Int('a_'+str(i)) for i in range(num_instrs)]

    # extract instruction sequence from Z3 model
    def instrs_in_seq(self, model, instrs):
        output_instrs = []
        for i in range(len(instrs)):
            output_instrs.append(model.eval(instrs[i]))
        return output_instrs

    # a convenience function for printing the output to look like a sequence of instructions
    def print_instrs(self, instrs, distances=None):
        printable_instrs = []
        for i in range(len(instrs)):
            val = instrs[i]
            distance_str = "" if distances == None else "("+str(distances[i])+")"
            if (val == 0):
                printable_instrs.append("L"+distance_str)
            elif (val == 1):
                printable_instrs.append("R"+distance_str)
            else:
                printable_instrs.append("-")
        print("\n".join(printable_instrs))

    def synthesize(self):
        num_instrs = 2 # example value.  shouldn't have hard-coded num_instrs in HW!
        instrs = self.gen_instrs(num_instrs) # generate BVs to represent instructions
        args = self.gen_args(num_instrs) # generate BVs to represent arguments
        goal = self.run_prog(self.start_x_coord, instrs, args, self.width) == self.goal_x_coord # where do we want our robot to move?

        s = Solver()
        s.add(goal)
        satisfiable = s.check()
        # print("satisfiable?", satisfiable)

        if (satisfiable == sat):
            model = s.model()
            instrs = self.instrs_in_seq(model, instrs) 
            distances = self.instrs_in_seq(model, args) 
            self.print_instrs(instrs, distances) # print the program if we found one
            # print(model) # print the model, just to visualize what's happening underneath
            return instrs

def parse_obstacles(str):
    coords = str.split(";")
    return [(int(comps[0]), int(comps[1])) for comps in [coord.split(",") for coord in coords]]

if __name__ == '__main__':
    # --------------------------------------------------------
    # Parse command line args
    # Note that several aren't used in this program but will be required for HW.
    # --------------------------------------------------------

    grid_width = int(sys.argv[1]) # how many squares across?
    grid_length = int(sys.argv[2]) # how many squares up and down?  (ignored in this prog, but required for HW)
    start_x_coord = int(sys.argv[3]) # with leftmost colum at coord 0, coord of start square
    start_y_coord = int(sys.argv[4]) # with topmost row at coord 0, coord of start square  (ignored in this prog, but required for HW)
    goal_x_coord = int(sys.argv[5]) # with leftmost colum at coord 0, coord of goal square
    goal_y_coord = int(sys.argv[6]) # with topmost row at coord 0, coord of goal square  (ignored in this prog, but required for HW)
    
    obstacles = []
    if len(sys.argv) >= 8:
        obstacles = parse_obstacles(sys.argv[7]) #(ignored in this prog, but required for HW)
    
    synth = Synthesizer(grid_width, grid_length, start_x_coord, start_y_coord, goal_x_coord, goal_y_coord, obstacles)
    synth.synthesize()

