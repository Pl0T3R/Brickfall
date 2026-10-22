# Brickfall

Mixed-reality brick smashing for Meta Quest 3. Virtual toy bricks and balls in your real room, played with hand tracking only (no controllers). It runs in the Quest Browser as a WebXR page, so there is nothing to install on the headset.

| File | What it does |
| --- | --- |
| `physics.js` | The physics engine, written for this project. No dependencies. |
| `main.js` | The app: hands, grabbing and throwing, wrist menu, room setup, piles, sound, rendering (three.js). |
| `index.html` | Start page with the "Enter mixed reality" button. Also a mouse preview on a PC. |

## Getting it onto the headset

WebXR only runs on `https://` pages (or `http://localhost`). Pick one:

1. **GitHub Pages:** push this folder to a GitHub repo, then go to Settings → Pages → Deploy from branch. Open `https://<you>.github.io/<repo>/` in the Quest Browser.
2. **Netlify Drop:** drag this folder onto <https://app.netlify.com/drop> and open the URL it gives you on the Quest.
3. **USB cable (developer mode on):** on your PC run `npx http-server . -p 8080` in this folder, then `adb reverse tcp:8080 tcp:8080`, and open `http://localhost:8080` in the Quest Browser.

Before you play, run **Space Setup** (Settings → Physical Space → Space Setup) so the floor, walls and tables are known. The first time you enter, the browser asks to share hand tracking and room data. Allow both.

## Playing

- **Get a ball:** pinch (thumb + index) or make a fist in empty air. A ball appears in your hand.
- **Throw:** swing and open your hand. The throw uses your hand's fastest motion in the last ~0.1 s.
- **Pick things up:** pinch or grab near a brick or ball. It lights up when it's in reach.
- **Push:** your fingertips and palm are solid, so you can sweep bricks off a table.
- **Menu:** turn either palm toward your face. Tap buttons with the other index finger.
  - Rebuild, Move pile (point at the floor or a tabletop, pinch to drop it there)
  - Pile: Wall / Pyramid / Tower / Castle
  - Ball: Normal / Big / Heavy / Bouncy
  - Gravity: Earth / Moon / Zero, Slow-mo, Hand push on/off
  - Room: show the detected planes (or "Scan room" if none were found), Exit MR

The first pile appears about 1.4 m in front of you on the floor. Detected tables and walls are solid, hide virtual objects behind them, and catch shadows.

## Tuning

Everything you'd want to change is at the top of `main.js`:

- `BRICK`: size (default 10 × 5 × 5 cm), mass, friction, bounciness
- `BALL_TYPES`: radius, mass, bounciness of each ball
- `pileLayout()`: the four pile shapes (rows, columns, layers)
- `GRAB_RANGE`, `THROW_BOOST`, `MAX_BALLS`

Engine settings are on the `World` in `physics.js` (`substeps`, `contactHertz`, `sleepThreshold`, `restitutionThreshold`).

## How the physics works

- **Shapes:** spheres, boxes, and infinite planes. Room surfaces become thin static boxes.
- **Collision:** sweep-and-prune broadphase. Box-box contact uses a separating-axis test with face clipping (up to 4 contact points). Speculative contacts stop fast balls from passing through bricks (tested up to 40 m/s).
- **Solver:** sequential impulses with soft contacts and substeps, the "soft step" method from Box2D v3. It uses warm starting, Coulomb friction, restitution, and rolling resistance for balls.
- **Sleeping:** piles at rest fall asleep and cost nothing until something touches them.
- **Hands:** each hand is 11 kinematic spheres (fingertips, finger joints, palm) that push objects but are never pushed back. A held object becomes kinematic and follows your wrist.

It runs 6 substeps per frame and drops to 4 automatically if a big collapse makes a step take longer than 4.5 ms.

## Testing done so far

- **Engine, in Node:** piles settle and sleep. A 60-brick wall and a 15-layer tower stand still, and a 114-brick wall stands too. Balls knock piles over, and fast balls don't tunnel.
- **Page, in headless Chromium:** the desktop preview runs with no errors.
- **Mixed-reality path, in Meta's IWER emulator with emulated Quest 3 hands:**
  - fist-grab spawns a ball, and opening the hand throws it
  - the wrist menu opens and responds to a finger poke
  - room planes turn into solid walls and tables
  - moving the pile onto a table works
  - a hand sweep pushes bricks
  - leaving and re-entering mixed reality works

It has not been run on a physical Quest 3 yet, so gesture thresholds and throw feel may need tuning on the real device.
