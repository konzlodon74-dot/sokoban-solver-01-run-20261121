# sokoban-priority — a push-optimal Sokoban solver

[![Bounty](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![Rank](https://make.fun/badge/sokoban-solver-01-run-20261121/konzlodon74-dot/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![Reward](https://make.fun/badge/sokoban-solver-01-run-20261121/konzlodon74-dot/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

An entry for make.fun's [sokoban-solver-01-run-20261121](https://make.fun/bounty/sokoban-solver-01-run-20261121)
("Build the best Sokoban solver", 21 November 2026): one program, `main.ts` at the repository root, that answers
every instance of a set — for each `/input/<id>/level.txt` (XSB) it writes `/output/<id>/solution.txt`, the player's
moves in LURD, solving the level in the fewest pushes it can prove, P\*.

## How it works

The levels this bounty draws are a one-square-wide hallway with independent rooms off it, each room holding 3–4
boxes in a 6×6 slot, and a box can never leave its room — so a level's push optimum is the sum of its rooms' optima.

1. **Split.** The level is cut into its rooms exactly the way the generator cuts them: the dead squares of the
   player's row (the hallway) are removed and each remaining part holding a box is a room. A level that is no set of
   rooms is searched whole instead.
2. **Solve each room push-optimally.** Rooms are taken in turn, the other rooms' boxes standing as walls, with an
   A\* search **over pushes, not moves**: a state is the boxes' squares plus the player's reachable region, and each
   push costs 1. The estimate is a **minimum-cost bipartite matching** (Hungarian algorithm) of boxes to goals over
   true push distances — each box's fewest pushes to each goal, computed by a breadth-first pull-back over the push
   graph — which is admissible, so with re-opening the first solution A\* returns is push-optimal.
3. **Prune.** A box pushed onto a dead square (no push from it reaches any goal) or into a frozen 2×2 block of walls
   and boxes with a box off its goal is discarded at once; a transposition table keeps each state's best push count.
4. **Stitch.** Between pushes the player's walk is a shortest path around the boxes; the rooms' move strings are
   concatenated, and the whole answer is replayed in-process as a self-check before it is written.

A breadth-first push search (the same shape as the generator's own) stands behind the A\* as a fallback. All budgets
are counted in states, never by the clock, and every tie breaks by square order, so two executions of the entry write
byte-identical answers.

## Local run

```
mkdir -p out
MAKEFUN_INPUT_DIR=<kit>/practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

On the kit's practice set the solver answers all five instances in exactly P\* pushes (13, 16, 29, 47, 60) — a
perfect 1,000,000 under the kit's scorer — in a fraction of a second for the whole set.

## Licence

MIT — see [LICENSE](LICENSE).
