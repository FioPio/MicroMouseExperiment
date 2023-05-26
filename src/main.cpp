#include "Simulator/Simulator.h"

int main() {

  Simulator::Maze maze;

  maze.drawMaze(10);

  Simulator::MicroMouse micro_mouse(&maze);

  micro_mouse.run();

  return 0;
}
