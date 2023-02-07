import * as THREE from "three";

class LinesBuffer {
  MAX_POINTS = 200000;

  constructor() {
    const positions = new Float32Array(this.MAX_POINTS * 3); // 3 vertices per point
    const colors = new Float32Array(this.MAX_POINTS * 4); // RGB for each point

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
      "position",
      new THREE.BufferAttribute(positions, 3).setUsage(THREE.StreamDrawUsage) // TODO(greg): try StreamDrawUsage
    );
    geometry.setAttribute(
      "color",
      new THREE.BufferAttribute(colors, 4).setUsage(THREE.StreamDrawUsage) // TODO(greg): try StreamDrawUsage
    );
    geometry.setDrawRange(0, 0); // initially draw nothing

    const material = new THREE.LineBasicMaterial({
      vertexColors: true,
      blending: THREE.AdditiveBlending,
      transparent: true,
    });

    this.lines = new THREE.LineSegments(geometry, material);
    this.lines.visible = true;
  }

  set_lines(line_list) {
    const positions = this.lines.geometry.attributes.position.array;
    const colors = this.lines.geometry.attributes.color.array;

    const indices = [];
    let count = 0; // one for each point in the geometry position buffer

    // for (let i = 0; i < line_list.length; i++) {
    for (let i = 0; i < line_list.size(); i++) {
      // const line = line_list[i];
      const line = line_list.get(i);
      const line_size = line.size();
      // TODO(greg): check for overflow wrt MAX_POINTS here

      // for (let k = 0; k < line.length; k++) {
      for (let k = 0; k < line_size; k++) {
        // if (k < line.length - 1) {
        if (k < line_size - 1) {
          indices.push(count, count + 1);
        }
        // const xyz_rgb = line[k];
        const xyz_rgb = line.get(k);

        positions[3 * count] = xyz_rgb.x;
        positions[3 * count + 1] = xyz_rgb.y;
        positions[3 * count + 2] = xyz_rgb.z;
        colors[4 * count] = xyz_rgb.r;
        colors[4 * count + 1] = xyz_rgb.g;
        colors[4 * count + 2] = xyz_rgb.b;
        colors[4 * count + 3] = xyz_rgb.a;

        // console.log("x: " + xyz_rgb.x);
        // console.log("y: " + xyz_rgb.y);
        // console.log("z: " + xyz_rgb.z);
        // console.log("r: " + xyz_rgb.r);
        // console.log("g: " + xyz_rgb.g);
        // console.log("b: " + xyz_rgb.b);
        count++;
      }
      line.delete();
    }
    // print indices
    // console.log("count: " + count);
    // console.log("indices: " + indices);
    this.lines.geometry.attributes.position.needsUpdate = true; // required after updates
    this.lines.geometry.attributes.color.needsUpdate = true; // required after updates
    this.lines.geometry.setIndex(indices);
    this.lines.geometry.setDrawRange(0, indices.length);
    this.lines.geometry.computeBoundingBox();
    this.lines.geometry.computeBoundingSphere();
  }

  set_lines_from_lists(line_list) {
    const positions = this.lines.geometry.attributes.position.array;
    const colors = this.lines.geometry.attributes.color.array;

    const indices = [];
    let count = 0; // one for each point in the geometry position buffer

    for (let i = 0; i < line_list.length; i++) {
      // for (let i = 0; i < line_list.size(); i++) {
      const line = line_list[i];
      //   const line = line_list.get(i);
      //   const line_size = line.size();
      const line_size = line.length;
      // TODO(greg): check for overflow wrt MAX_POINTS here

      //   for (let k = 0; k < line.length; k++) {
      for (let k = 0; k < line_size; k++) {
        // if (k < line.length - 1) {
        if (k < line_size - 1) {
          indices.push(count, count + 1);
        }
        const xyz_rgb = line[k];
        // const xyz_rgb = line.get(k);

        positions[3 * count] = xyz_rgb.x;
        positions[3 * count + 1] = xyz_rgb.y;
        positions[3 * count + 2] = xyz_rgb.z;
        colors[4 * count] = xyz_rgb.r;
        colors[4 * count + 1] = xyz_rgb.g;
        colors[4 * count + 2] = xyz_rgb.b;
        colors[4 * count + 3] = xyz_rgb.a;

        count++;
      }
    }
    this.lines.geometry.attributes.position.needsUpdate = true; // required after updates
    this.lines.geometry.attributes.color.needsUpdate = true; // required after updates
    this.lines.geometry.setIndex(indices);
    this.lines.geometry.setDrawRange(0, indices.length);
    this.lines.geometry.computeBoundingBox();
    this.lines.geometry.computeBoundingSphere();
  }

  get_segments() {
    return this.lines;
  }
}

export { LinesBuffer };
