// sokoban-priority 1.0.0: a push-optimal Sokoban solver for make.fun's sokoban-solver-01 (MIT, LICENSE beside this
// file). The levels the generator draws are a hallway with independent rooms off it, so the solver splits a level
// into its rooms (the generator's own cut: the dead squares of the player's row) and solves each room in turn with
// the other rooms' boxes standing as walls; a room's optimum is found by A* over pushes, never over moves, guided
// by a minimum-cost bipartite matching of boxes to goals over true push distances, so the pushes of the rooms add
// up to the level's push optimum P*. Dead squares (no push from them reaches a goal) and frozen 2 x 2 blocks prune
// the search; a transposition table keeps each state's best push count. Work is budgeted in states, never by the
// clock, and every tie breaks by square order, so two runs write the same bytes.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

/** The most states one A* search generates before it gives up: work, never the clock. */
const ASTAR_STATES = 1_000_000;
/** The most states the fallback breadth-first search keeps: work, never the clock. */
const BFS_STATES = 1_000_000;

/** The four directions: [dx, dy, the move's letter, the push's letter]. */
const DIRS = [
  [0, -1, 'u', 'U'],
  [0, 1, 'd', 'D'],
  [-1, 0, 'l', 'L'],
  [1, 0, 'r', 'R'],
];

/** A level in XSB text: width and height, walls (the outermost frame is wall whatever it shows), goals, boxes, player. */
function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals = [];
  const boxes = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
  return { w, h, wall, goals, boxes: boxes.sort((a, b) => a - b), player };
}

/** The squares from which a box alone can be pushed onto some goal (pulled back from the goals over open floor). */
function liveSquares(level) {
  const { w, wall, goals } = level;
  const live = new Uint8Array(wall.length);
  const queue = [...goals];
  for (const g of goals) live[g] = 1;
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi];
    for (const [dx, dy] of DIRS) {
      const d = dy * w + dx;
      const y = x - d;
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

/**
 * Push distances to one goal: dist[s] is the fewest pushes that bring a box from s to the goal, other boxes ignored
 * (the player is wherever a push needs them), or -1 when no push brings it there. Breadth-first over box squares.
 */
function pushDistances(level, goalPos) {
  const { w, wall } = level;
  const dist = new Int32Array(wall.length).fill(-1);
  dist[goalPos] = 0;
  const queue = [goalPos];
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi];
    for (const [dx, dy] of DIRS) {
      const d = dy * w + dx;
      const y = x - d;
      if (dist[y] < 0 && !wall[y] && !wall[y - d]) {
        dist[y] = dist[x] + 1;
        queue.push(y);
      }
    }
  }
  return dist;
}

const INF = 1e9;

/**
 * The minimum-cost matching of boxes to goals (Hungarian algorithm, O(n^3)) over a cost matrix; a cost of INF or
 * more in the result means some box reaches no goal. Admissible as a push estimate: every box must travel at least
 * its own push distance, and two boxes cannot share a goal.
 */
function matchingCost(cost) {
  const n = cost.length;
  if (n === 0) return 0;
  const u = new Float64Array(n + 1);
  const v = new Float64Array(n + 1);
  const p = new Int32Array(n + 1);
  const way = new Int32Array(n + 1);
  for (let i = 1; i <= n; i++) {
    p[0] = i;
    let j0 = 0;
    const minv = new Float64Array(n + 1).fill(INF);
    const used = new Array(n + 1).fill(false);
    do {
      used[j0] = true;
      const i0 = p[j0];
      let delta = INF;
      let j1 = -1;
      for (let j = 1; j <= n; j++) {
        if (used[j]) continue;
        const cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (let j = 0; j <= n; j++) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else minv[j] -= delta;
      }
      j0 = j1;
    } while (p[j0] !== 0);
    do {
      const j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 !== 0);
  }
  let total = 0;
  for (let j = 1; j <= n; j++) total += cost[p[j] - 1][j - 1];
  return total;
}

/** The squares the player reaches from `from` with `occupied` in the way, marked in `seen` with `stamp`; the lowest. */
function reach(level, occupied, from, seen, stamp, stack) {
  const { w, wall } = level;
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n];
    if (p < low) low = p;
    const up = p - w;
    const dn = p + w;
    const lf = p - 1;
    const rt = p + 1;
    if (wall[up] === 0 && occupied[up] === 0 && seen[up] !== stamp) { seen[up] = stamp; stack[n++] = up; }
    if (wall[dn] === 0 && occupied[dn] === 0 && seen[dn] !== stamp) { seen[dn] = stamp; stack[n++] = dn; }
    if (wall[lf] === 0 && occupied[lf] === 0 && seen[lf] !== stamp) { seen[lf] = stamp; stack[n++] = lf; }
    if (wall[rt] === 0 && occupied[rt] === 0 && seen[rt] !== stamp) { seen[rt] = stamp; stack[n++] = rt; }
  }
  return low;
}

/** Whether the box just pushed to `t` sits in a frozen 2 x 2 block of walls and boxes with a box off its goal. */
function frozenBlock(level, occupied, goal, t) {
  const { w, wall } = level;
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (!(wall[a] || occupied[a]) || !(wall[b] || occupied[b]) || !(wall[c] || occupied[c]) || !(wall[d] || occupied[d])) continue;
    if ((occupied[a] && !goal[a]) || (occupied[b] && !goal[b]) || (occupied[c] && !goal[c]) || (occupied[d] && !goal[d])) return true;
  }
  return false;
}

/** The shortest walk from `from` to `to` around walls and boxes, in lower LURD letters (the first direction first on ties). */
function walkTo(level, occupied, from, to) {
  const { w, wall } = level;
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Array(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi];
    if (p === to) break;
    for (const [dx, dy, letter] of DIRS) {
      const q = p + dy * w + dx;
      if (wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = letter;
      queue.push(q);
    }
  }
  if (prev[to] === -1) throw new Error('walkTo: the push square is out of reach');
  let s = '';
  for (let p = to; p !== from; p = prev[p]) s = how[p] + s;
  return s;
}

/**
 * The push-optimal solution of `level` by A* over pushes, or null when the search gives up: {pushes, moves,
 * endPlayer}. The estimate is the box-to-goal matching cost; a state's key is its boxes plus the lowest square of
 * the player's reachable region.
 */
function solvePushAStar(level, maxStates) {
  const { w, wall, goals } = level;
  const size = wall.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(level);
  if (level.boxes.some((b) => !live[b])) return null;
  const dists = goals.map((g) => pushDistances(level, g));
  const nb = level.boxes.length;
  // The matching runs on a square matrix of side max(boxes, goals): a spare goal takes a dummy box at no cost and a
  // spare box a dummy goal, so the estimate stays a relaxation whatever the counts (the generator draws them equal).
  const side = Math.max(nb, goals.length);
  const cost = Array.from({ length: side }, () => new Array(side).fill(INF));
  const estimate = (boxes) => {
    for (let i = 0; i < side; i++) {
      for (let j = 0; j < side; j++) {
        if (i >= nb || j >= goals.length) {
          cost[i][j] = 0;
        } else {
          const d = dists[j][boxes[i]];
          cost[i][j] = d < 0 ? INF : d;
        }
      }
    }
    return matchingCost(cost);
  };
  const solved = (boxes) => boxes.every((b) => goal[b]);
  if (solved(level.boxes)) return { pushes: 0, moves: '', endPlayer: level.player };

  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;

  // The states: boxes, the player's square in the state, its key, pushes so far, parent, and the segment that made it.
  const statesBoxes = [level.boxes];
  const statesPlayer = [level.player];
  const statesSeg = [''];
  const parent = [-1];
  const best = new Map();
  for (const b of level.boxes) occupied[b] = 1;
  const rootLow = reach(level, occupied, level.player, seen, ++stamp, stack);
  for (const b of level.boxes) occupied[b] = 0;
  const keyOf = (boxes, low) => String.fromCharCode(low, ...boxes);
  const rootKey = keyOf(level.boxes, rootLow);
  const statesKey = [rootKey];
  best.set(rootKey, 0);

  // The open list: a heap on (f, then more pushes first, then the older state first).
  const heap = [];
  const less = (a, b) => a[0] < b[0] || (a[0] === b[0] && (a[1] > b[1] || (a[1] === b[1] && a[2] < b[2])));
  const hpush = (e) => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      [heap[i], heap[p]] = [heap[p], heap[i]];
      i = p;
    }
  };
  const hpop = () => {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1], heap[c])) c++;
        if (!less(heap[c], heap[i])) break;
        [heap[i], heap[c]] = [heap[c], heap[i]];
        i = c;
      }
    }
    return top;
  };
  hpush([estimate(level.boxes), 0, 0]);

  let found = -1;
  while (heap.length > 0 && found < 0) {
    const [, g, s] = hpop();
    const boxes = statesBoxes[s];
    // Skip a stale heap entry: a cheaper path to this state was recorded after the entry was queued.
    if (best.get(statesKey[s]) < g) continue;
    if (solved(boxes)) {
      found = s;
      break;
    }
    for (const b of boxes) occupied[b] = 1;
    reach(level, occupied, statesPlayer[s], seen, ++stamp, stack);
    const here = stamp;
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi];
      for (const [dx, dy, , pushLetter] of DIRS) {
        const d = dy * w + dx;
        const t = b + d;
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(level, occupied, goal, t);
        let key = null;
        let next = null;
        let low = -1;
        if (!frozen) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((x, y) => x - y);
          low = reach(level, occupied, b, mark, ++stamp, stack);
          key = keyOf(next, low);
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen) continue;
        const ng = g + 1;
        const seenG = best.get(key);
        if (seenG !== undefined && seenG <= ng) continue;
        if (statesBoxes.length >= maxStates) {
          for (const x of boxes) occupied[x] = 0;
          return null;
        }
        const h = estimate(next);
        if (h >= INF) continue;
        best.set(key, ng);
        const t2 = statesBoxes.length;
        statesBoxes.push(next);
        statesPlayer.push(b);
        statesKey.push(key);
        statesSeg.push(walkTo(level, occupied, statesPlayer[s], b - d) + pushLetter);
        parent.push(s);
        hpush([ng + h, ng, t2]);
      }
    }
    for (const x of boxes) occupied[x] = 0;
  }
  if (found < 0) return null;
  const segs = [];
  for (let s = found; s > 0; s = parent[s]) segs.push(statesSeg[s]);
  segs.reverse();
  let pushes = 0;
  for (let s = found; s > 0; s = parent[s]) pushes++;
  return { pushes, moves: segs.join(''), endPlayer: statesPlayer[found] };
}

/**
 * The push-optimal solution of `level` by breadth-first search over pushes (the generator's own method, kept as a
 * fallback): {pushes, moves, endPlayer}, or null when the search gives up.
 */
function solvePushBFS(level, maxStates) {
  const { w, wall, goals } = level;
  const size = wall.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(level);
  if (level.boxes.some((b) => !live[b])) return null;
  const solved = (boxes) => boxes.every((b) => goal[b]);
  if (solved(level.boxes)) return { pushes: 0, moves: '', endPlayer: level.player };
  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;
  const keyOf = (boxes, low) => String.fromCharCode(low, ...boxes);
  const statesBoxes = [level.boxes];
  const statesPlayer = [level.player];
  const statesSeg = [''];
  const parent = [-1];
  const index = new Map();
  for (const b of level.boxes) occupied[b] = 1;
  index.set(keyOf(level.boxes, reach(level, occupied, level.player, seen, ++stamp, stack)), 0);
  for (const b of level.boxes) occupied[b] = 0;
  let found = -1;
  for (let head = 0; found < 0 && head < statesBoxes.length; head++) {
    const boxes = statesBoxes[head];
    for (const b of boxes) occupied[b] = 1;
    reach(level, occupied, statesPlayer[head], seen, ++stamp, stack);
    const here = stamp;
    for (let bi = 0; bi < boxes.length && found < 0; bi++) {
      const b = boxes[bi];
      for (const [dx, dy, , pushLetter] of DIRS) {
        const d = dy * w + dx;
        const t = b + d;
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(level, occupied, goal, t);
        let key = null;
        let next = null;
        if (!frozen) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((x, y) => x - y);
          key = keyOf(next, reach(level, occupied, b, mark, ++stamp, stack));
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen || index.has(key)) continue;
        if (statesBoxes.length >= maxStates) {
          for (const x of boxes) occupied[x] = 0;
          return null;
        }
        index.set(key, statesBoxes.length);
        statesBoxes.push(next);
        statesPlayer.push(b);
        statesSeg.push(walkTo(level, occupied, statesPlayer[head], b - d) + pushLetter);
        parent.push(head);
        if (solved(next)) found = statesBoxes.length - 1;
      }
    }
    for (const x of boxes) occupied[x] = 0;
  }
  if (found < 0) return null;
  const segs = [];
  for (let s = found; s > 0; s = parent[s]) segs.push(statesSeg[s]);
  segs.reverse();
  return { pushes: segs.length, moves: segs.join(''), endPlayer: statesPlayer[found] };
}

/** One push search: A* first, the breadth-first search as its fallback. */
function solvePush(level, maxStates) {
  return solvePushAStar(level, maxStates) ?? solvePushBFS(level, maxStates);
}

/**
 * The level's rooms: cut the dead squares of the player's row (the hallway: a box there reaches no goal) out of the
 * floor, and each part left that holds a box is a room. Null when the level is no set of rooms.
 */
function roomsOf(level) {
  const { w, wall, goals, boxes, player } = level;
  const live = liveSquares(level);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) if (!wall[i] && !live[i]) cut[i] = 1;
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (const [dx, dy] of DIRS) {
        const q = squares[i] + dy * w + dx;
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ squares, boxes: [], goals: [] });
  }
  for (const b of boxes) rooms[part[b]].boxes.push(b);
  for (const g of goals) if (part[g] >= 0) rooms[part[g]].goals.push(g);
  if (goals.some((g) => part[g] < 0) || rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

/** Replay `moves` on the level from its start: {solved, pushes} of the final position (an illegal step fails). */
function replay(level, moves) {
  const { w, wall } = level;
  const box = new Uint8Array(wall.length);
  for (const b of level.boxes) box[b] = 1;
  const goal = new Uint8Array(wall.length);
  for (const g of level.goals) goal[g] = 1;
  let player = level.player;
  let pushes = 0;
  for (const ch of moves.toLowerCase()) {
    const d = ch === 'u' ? -w : ch === 'd' ? w : ch === 'l' ? -1 : 1;
    const next = player + d;
    if (wall[next]) return { solved: false, pushes };
    if (box[next]) {
      const beyond = next + d;
      if (wall[beyond] || box[beyond]) return { solved: false, pushes };
      box[next] = 0;
      box[beyond] = 1;
      pushes++;
    }
    player = next;
  }
  for (let i = 0; i < box.length; i++) if (box[i] && !goal[i]) return { solved: false, pushes };
  return { solved: true, pushes };
}

/** The level solved room by room, each room push-optimal in turn; null when a room's search gives up. */
function solveByRooms(level) {
  const rooms = roomsOf(level);
  if (rooms === null) return null;
  let player = level.player;
  let moves = '';
  for (const room of rooms) {
    const wall = level.wall.slice();
    for (const other of rooms) if (other !== room) for (const b of other.done ? other.goals : other.boxes) wall[b] = 1;
    const sub = { w: level.w, h: level.h, wall, goals: room.goals.slice().sort((a, b) => a - b), boxes: room.boxes.slice().sort((a, b) => a - b), player };
    const solvedRoom = solvePush(sub, ASTAR_STATES);
    if (solvedRoom === null) return null;
    moves += solvedRoom.moves;
    player = solvedRoom.endPlayer;
    room.done = true;
  }
  return moves;
}

/** The moves that solve the level in as few pushes as the search can prove, or '' when it gives up. */
function solve(text) {
  const level = parseLevel(text);
  if (level.boxes.length === 0 || level.player < 0) return '';
  const attempts = [];
  const byRooms = solveByRooms(level);
  if (byRooms !== null) attempts.push(byRooms);
  if (attempts.length === 0 || !replay(level, attempts[0]).solved) {
    const whole = solvePush(level, ASTAR_STATES);
    if (whole !== null) attempts.push(whole.moves);
  }
  for (const moves of attempts) if (replay(level, moves).solved) return moves;
  return '';
}

for (const id of readdirSync(I).sort()) {
  const moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
