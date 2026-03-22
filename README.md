# cs294-2026-a5

Z3-based robot motion synthesizer that finds the shortest instruction sequence to move a robot from a start position to a goal on a 2D grid, navigating around obstacles.

## Setup

```
pip install -r requirements.txt
```

## Usage

```
python3 synthesizer.py <width> <height> <start_x> <start_y> <goal_x> <goal_y> "<obstacles>"
```

Obstacles are semicolon-separated `x,y` pairs. Pass `""` for no obstacles.

### Examples

```
python3 synthesizer.py 4 4 0 0 2 0 ""          # horizontal move
python3 synthesizer.py 4 4 0 0 2 2 ""          # diagonal move
python3 synthesizer.py 5 5 0 0 4 0 "2,0;2,1;2,2"  # navigate around wall
```

## Instructions

The synthesizer outputs one instruction per line: `L` (left), `R` (right), `D` (down), `U` (up). It uses iterative deepening to find the shortest path.
