import { GUI } from "three/addons/libs/lil-gui.module.min.js";
import wasmModule from "bazel-bin/vis/shim_wasm/shim.js";

class Arrow {
  constructor(name, x, y, angle_deg) {
    this.name = name; // for debug messages
    this.x = x;
    this.y = y;
    this.angle = angle_deg * (Math.PI / 180);
    this.length = 40;
  }

  moveRoot(x, y) {
    this.x = x;
    this.y = y;
  }

  moveHead(x, y) {
    const dx = x - this.x;
    const dy = y - this.y;
    this.angle = Math.atan2(dy, dx);
  }

  distanceToRoot(x, y) {
    return Math.sqrt((x - this.x) ** 2 + (y - this.y) ** 2);
  }

  distanceToHead(x, y) {
    const head_x = this.x + this.length * Math.cos(this.angle);
    const head_y = this.y + this.length * Math.sin(this.angle);
    return Math.sqrt((x - head_x) ** 2 + (y - head_y) ** 2);
  }

  draw(context) {
    // console.log("drawing " + this.name);
    context.save();
    context.translate(this.x, this.y);
    context.rotate(this.angle);

    // draw line
    context.lineWidth = 2;
    context.beginPath();
    context.moveTo(0, 0);
    context.lineTo(this.length, 0);
    context.stroke();

    // draw arrowhead
    context.translate(this.length, 0);
    context.beginPath();
    context.moveTo(0, 0);
    context.lineTo(0, -5);
    context.lineTo(10, 0);
    context.lineTo(0, 5);
    context.lineTo(0, 0);
    context.fill();

    context.restore();
  }
}

class DubinsPath {
  constructor(cxxModule, pathType) {
    this.path = new cxxModule.DubinsPath();
    this.pathType = pathType;
    this.line = [];
    this.infoString = "";
  }

  isValid() {
    return this.line.length != 0;
  }

  update(q0, q1, params, cxxModule, intermediateResults) {
    // console.log(`updating path ${this.pathType.constructor.name}`);
    const wordResult = cxxModule.ComputeDubinsPath(
      this.path,
      q0,
      q1,
      params.rho,
      this.pathType,
      intermediateResults
    );

    // Draw line if successful, otherwise leave line empty.
    this.line = [];
    if (wordResult === cxxModule.DubinsWordStatus.kSuccess) {
      const totalLength = this.path.TotalLength();
      const n_samples = params.num_samples;
      for (let i = 0; i < n_samples; i++) {
        const t = 0.99999 * (i / (n_samples - 1)) * totalLength;
        const coord = this.path.Sample(t);
        this.line.push({
          x: coord.position.x,
          y: coord.position.y,
        });
      }
      this.infoString = totalLength.toFixed(2);
    } else {
      this.infoString = `${wordResult.constructor.name}`;
    }
  }

  draw(contex, color) {
    if (this.line.length < 2) {
      return;
    }
    context.save();
    context.lineWidth = 2;
    context.beginPath();
    context.strokeStyle = color;
    context.moveTo(this.line[0].x, this.line[0].y);
    for (let i = 1; i < this.line.length; i++) {
      context.lineTo(this.line[i].x, this.line[i].y);
    }
    context.stroke();
    context.restore();
  }
}

class DubinsPaths {
  constructor(cxxModule, pathTypes) {
    this.paths = [];
    for (const pathType of pathTypes) {
      this.paths.push(new DubinsPath(cxxModule, pathType));
    }
  }

  bestPathIndex() {
    let bestPathIndex = -1;
    let bestPathLength = Infinity;
    for (let i = 0; i < this.paths.length; i++) {
      if (this.paths[i].isValid()) {
        const pathLength = this.paths[i].path.TotalLength();
        if (pathLength < bestPathLength) {
          bestPathIndex = i;
          bestPathLength = pathLength;
        }
      }
    }
    if (bestPathIndex < 0) {
      console.warn("no valid paths");
    }
    return bestPathIndex;
  }

  update(startArrow, endArrow, params, cxxModule) {
    const q0 = {
      position: { x: startArrow.x, y: startArrow.y },
      theta: startArrow.angle,
    };
    const q1 = {
      position: { x: endArrow.x, y: endArrow.y },
      theta: endArrow.angle,
    };
    const intermediateResults = cxxModule.ComputeDubinsIntermediateResults(
      q0,
      q1,
      params.rho
    );

    for (const path of this.paths) {
      path.update(q0, q1, params, cxxModule, intermediateResults);
    }

    intermediateResults.delete();
  }

  draw(context, colors, params) {
    // assert length of colors is equal to length of paths
    if (colors.length != this.paths.length) {
      console.warn("length of colors and paths must be equal");
      return;
    }

    const bestPathIndex = this.bestPathIndex();

    // only draw best path, if that option is specified
    if (params.only_draw_best_path) {
      context.setLineDash([]);
      this.paths[bestPathIndex].draw(context, colors[bestPathIndex]);
      return;
    }

    // draw paths
    for (let i = 0; i < this.paths.length; i++) {
      if (i == bestPathIndex) {
        context.setLineDash([]);
      } else {
        context.setLineDash([5, 15]);
      }
      this.paths[i].draw(context, colors[i]);
    }
  }
}

// load webassembly module
let cxxModule = await wasmModule({
  onRuntimeInitialized() {
    console.log("WASM module initialized.");
  },
});
let canvas;
let context;

const startArrow = new Arrow("start", 154, 433, -25.1);
const endArrow = new Arrow("end", 250, 250, -61.8);

const dubinsPaths = new DubinsPaths(cxxModule, [
  cxxModule.DubinsPathType.kLsl,
  cxxModule.DubinsPathType.kLsr,
  cxxModule.DubinsPathType.kRsl,
  cxxModule.DubinsPathType.kRsr,
  cxxModule.DubinsPathType.kRlr,
  cxxModule.DubinsPathType.kLrl,
]);

const params = {
  rho: 50,
  num_samples: 100,
  only_draw_best_path: false,
};

const colors = [
  "red",
  "green",
  "blue",
  "orange",
  "purple",
  "deepPink",
  // "black",
  // "brown",
];

function compute_and_draw() {
  // update dubins paths
  dubinsPaths.update(startArrow, endArrow, params, cxxModule);

  // update the info
  const info_strings = [];
  // append length of each path
  for (let i = 0; i < dubinsPaths.paths.length; i++) {
    const path = dubinsPaths.paths[i];
    const color = colors[i];
    const info = path.infoString;
    info_strings.push(
      `<font style="color:${color};">${path.pathType.constructor.name}: ${info}</font>`
    );
  }

  const info_string = info_strings.join("<br />");
  document.getElementById("info").innerHTML = info_string;

  // resize canvas
  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight;

  // clear canvas
  context.save();
  context.setTransform(1, 0, 0, 1, 0, 0);
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.restore();

  // draw stuff here
  // console.log("drawing");
  startArrow.draw(context);
  endArrow.draw(context);

  dubinsPaths.draw(context, colors, params);
}

let dragging = "none";

// window.onload = function () {
// console.log("onload");
canvas = document.getElementById("draw_area");
context = canvas.getContext("2d");

canvas.addEventListener("mousedown", function (event) {
  const x = event.offsetX;
  const y = event.offsetY;
  const test_points = [
    ["start_root", startArrow.distanceToRoot(x, y)],
    ["start_head", startArrow.distanceToHead(x, y)],
    ["end_root", endArrow.distanceToRoot(x, y)],
    ["end_head", endArrow.distanceToHead(x, y)],
  ];

  dragging = "none";
  let best_distance = 1000000;
  const dragging_threshold = 20;
  for (const [name, distance] of test_points) {
    if (distance < dragging_threshold && distance < best_distance) {
      dragging = name;
      best_distance = distance;
    }
  }

  // print debug message if dragging is not "none"
  if (dragging != "none") {
    console.log("dragging " + dragging);
  }
});

canvas.addEventListener("mouseup", function (event) {
  if (dragging === "start_root") {
    console.log(`dragged start to (${startArrow.x}, ${startArrow.y})`);
  } else if (dragging === "end_root") {
    console.log(`dragged end to (${endArrow.x}, ${endArrow.y})`);
  } else if (dragging === "start_head") {
    console.log(
      `start angle: ${(startArrow.angle * (180 / Math.PI)).toFixed(1)} deg`
    );
  } else if (dragging === "end_head") {
    console.log(
      `end angle: ${(endArrow.angle * (180 / Math.PI)).toFixed(1)} deg`
    );
  }
  // console.log("mouseup");
  // console.log(event);
  dragging = "none";
});

canvas.addEventListener("mousemove", function (event) {
  // console.log("mousemove");
  // console.log(event);
  if (dragging == "start_root") {
    startArrow.moveRoot(event.offsetX, event.offsetY);
  } else if (dragging == "end_root") {
    endArrow.moveRoot(event.offsetX, event.offsetY);
  } else if (dragging == "start_head") {
    startArrow.moveHead(event.offsetX, event.offsetY);
  } else if (dragging == "end_head") {
    endArrow.moveHead(event.offsetX, event.offsetY);
  }
  if (dragging != "none") {
    dubinsPaths.update(startArrow, endArrow, params, cxxModule);
  }
  compute_and_draw();
});

// draw initial state
compute_and_draw();
// };

let gui;
initGui();

function initGui() {
  gui = new GUI();

  gui.add(params, "rho", 1, 100, 1).onChange(function (value) {
    compute_and_draw();
  });
  gui.add(params, "num_samples", 2, 200, 1).onChange(function (value) {
    compute_and_draw();
  });
  gui.add(params, "only_draw_best_path").onChange(function (value) {
    compute_and_draw();
  });
  // gui.add(gui_params, "space", ["r3", "se2", "xyzq"]);
  // gui.add(gui_params, "delay_before_restart", 0.1, 5, 0.1);

  // for (const problem_type of ["r3_problem", "se2_problem", "xyzq_problem"]) {
  //   const problem_folder = gui.addFolder(problem_type);
  //   const problem_params = gui_params[problem_type];
  //   if (problem_type == "se2_problem") {
  //     problem_folder.add(problem_params, "rho", 0.01, 1.5, 0.01);
  //   }
  //   if (problem_type == "xyzq_problem") {
  //     problem_folder.add(problem_params, "rho", 0.01, 1.5, 0.01);
  //     problem_folder.add(problem_params, "max_glideslope", 0.5, 15, 0.5);
  //   }
  //   const max_eta = { r3_problem: 1, se2_problem: 10, xyzq_problem: 10 }[
  //     problem_type
  //   ];
  //   problem_folder.add(problem_params, "eta", 0, max_eta, 0.01);
  //   problem_folder.add(problem_params, "max_num_obstacles", 0, 150, 1);
  //   problem_folder.add(problem_params, "obstacle_fraction", 0, 1, 0.01);
  //   const min_length = problem_folder.add(
  //     problem_params,
  //     "min_length",
  //     0.05,
  //     9.95,
  //     0.05
  //   );
  //   const max_length = problem_folder.add(
  //     problem_params,
  //     "max_length",
  //     0.1,
  //     10,
  //     0.05
  //   );
  //   // make sure max_length is bigger than min_length
  //   min_length.onChange(function (min_length_val) {
  //     const max_length_val = max_length.getValue();
  //     if (min_length_val > max_length_val) {
  //       max_length.setValue(min_length_val + 0.05);
  //       max_length.updateDisplay();
  //     }
  //   });
  //   // make sure min length is smaller than max length
  //   max_length.onChange(function (max_length_val) {
  //     const min_length_val = min_length.getValue();
  //     if (max_length_val < min_length_val) {
  //       min_length.setValue(max_length_val - 0.05);
  //       min_length.updateDisplay();
  //     }
  //   });
  //   problem_folder.add(problem_params, "goal_radius", 0.01, 2, 0.01);
  //   problem_folder.add(problem_params, "max_iterations", 0, 10000);
  //   problem_folder.add(problem_params, "iterations_per_frame", 1, 500, 1);
  // }

  // const scene_folder = gui.addFolder("scene");
  // scene_folder.add(gui_params.scene, "rotate");
  // scene_folder.add(gui_params.scene, "rotation_rate", 0, 1.5);
  // scene_folder.add(gui_params.scene, "show_goal_region");
  // scene_folder.add(gui_params.scene, "show_obstacles");
  // scene_folder.add(gui_params.scene, "goal_region_opacity", 0, 1, 0.01);
  // scene_folder.add(gui_params.scene, "obstacle_opacity", 0, 1, 0.01);
  // scene_folder.add(gui_params.scene, "show_bounding_box");
  // scene_folder.add(gui_params.scene, "bounding_box_opacity", 0, 1, 0.01);
  // scene_folder.addColor(gui_params.scene, "optimal_line_color");
  // scene_folder.add(gui_params.scene, "show_axes");
}
