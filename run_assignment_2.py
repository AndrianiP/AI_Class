from argparse import ArgumentParser
from environments.connect_four import ConnectFourState, ConnectFour
from visualizer.connect_four_visualizer import ConnectFourVisualizer
import numpy as np

from coding_hw_answers.coding_hw2 import heuristic_minimax_search


def main():
    parser: ArgumentParser = ArgumentParser()
    parser.add_argument('--opponent', type=str, default="random", help="random or minimax")
    parser.add_argument('--depth', type=int, default=1, help="")

    args = parser.parse_args()

    assert args.depth > 0, "Depth must be greater than 0"

    env: ConnectFour = ConnectFour()

    if args.opponent == "random":
        def opponent(state: ConnectFourState):
            return np.random.choice(env.get_actions(state))
    elif args.opponent == "minimax":
        def opponent(state: ConnectFourState):
            return heuristic_minimax_search(state, env, args.depth)

    else:
        raise ValueError("Unknown opponent type %s" % args.opponent)

    viz = ConnectFourVisualizer(env, opponent)
    viz.mainloop()


if __name__ == "__main__":
    main()
