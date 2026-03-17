import * as THREE from "three";
import { FBXLoader } from "three/addons/loaders/FBXLoader.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const container = document.getElementById("modelContainer");
if (container) {
  const scene = new THREE.Scene();

  const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 1000);
  camera.position.set(3, 2, 5);

  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: "high-performance" });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
  renderer.setClearColor(0xffffff, 0);
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.sortObjects = false;
  container.appendChild(renderer.domElement);
  renderer.domElement.style.borderRadius = "8px";

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.enablePan = true;
  controls.autoRotate = false;
  controls.minDistance = 1;
  controls.maxDistance = 20;
  controls.minPolarAngle = Math.PI / 2 - 0.5;
  controls.maxPolarAngle = Math.PI / 2 + 0.3;

  scene.add(new THREE.AmbientLight(0xffffff, 0.6));
  const dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
  dirLight.position.set(5, 8, 5);
  scene.add(dirLight);
  const dirLight2 = new THREE.DirectionalLight(0x88bbff, 0.4);
  dirLight2.position.set(-3, -2, -4);
  scene.add(dirLight2);
  const rimLight = new THREE.DirectionalLight(0xffffff, 0.4);
  rimLight.position.set(0, -3, 5);
  scene.add(rimLight);

  let modelWrapper = null;

  const finPivots = [null, null, null, null];

  const thrusterPivots = [];

  const MAX_FIN_RAD = Math.PI / 6;

  const MAX_BLADE_RAD_PER_FRAME = (4 * Math.PI) / 60;


  const loadingEl = document.createElement("div");
  loadingEl.style.cssText = `
    position:absolute; inset:0; display:flex; align-items:center;
    justify-content:center; color:rgba(255,255,255,0.4);
    font-family:Inter,sans-serif; font-size:12px; letter-spacing:.08em;
    pointer-events:none;
  `;
  loadingEl.textContent = "LOADING MODEL…";
  container.style.position = "relative";
  container.appendChild(loadingEl);

  function pivotFromMesh(mesh) {
    if (!mesh.geometry) return mesh;

    mesh.updateWorldMatrix(true, false);

    mesh.geometry.computeBoundingBox();
    const localCentre = mesh.geometry.boundingBox.getCenter(new THREE.Vector3());
    const worldCentre = localCentre.clone().applyMatrix4(mesh.matrixWorld);

    const parent = mesh.parent;
    const parentInv = new THREE.Matrix4().copy(parent.matrixWorld).invert();
    const pivotLocalPos = worldCentre.clone().applyMatrix4(parentInv);

    mesh.position.sub(pivotLocalPos);

    const pivot = new THREE.Group();
    pivot.position.copy(pivotLocalPos);
    parent.remove(mesh);
    pivot.add(mesh);
    parent.add(pivot);
    return pivot;
  }

  const FIN_KW = ["fin", "dive", "plane", "aileron", "control", "rudder",
    "bow", "stern", "wing", "flap", "stab", "elevator"];
  const BLADE_KW = ["blade", "prop", "fan", "thruster", "rotor", "screw",
    "spin", "impel", "nozzle"];

  const loader = new FBXLoader();
  loader.load(
    "SUB.fbx",
    (fbx) => {
      loadingEl.remove();

      console.log("[Model] === Submarine_Fins_Blade.fbx — node hierarchy ===");
      fbx.traverse((node) => {
        console.log(`  [${node.type}] "${node.name}"${node.isMesh ? " ◀ MESH" : ""}`);
      });

      const fins = [];
      const blades = [];

      fbx.traverse((node) => {
        if (!node.isMesh) return;
        const n = node.name.toLowerCase();
        if (BLADE_KW.some(k => n.includes(k))) {
          blades.push(node);
        } else if (FIN_KW.some(k => n.includes(k))) {
          fins.push(node);
        }
      });

      console.log(`[Model] Fin candidates  (${fins.length}):`, fins.map(n => n.name));
      console.log(`[Model] Blade candidates (${blades.length}):`, blades.map(n => n.name));

      if (fins.length === 4) {
        fins.sort((a, b) => {
          const pA = new THREE.Vector3(); a.getWorldPosition(pA);
          const pB = new THREE.Vector3(); b.getWorldPosition(pB);
          if (Math.abs(pA.z - pB.z) > 0.01) return pA.z - pB.z;
          return pA.x - pB.x;
        });
        for (let i = 0; i < 4; i++) {
          finPivots[i] = pivotFromMesh(fins[i]);
          console.log(`[Model] finPivots[${i}] → "${fins[i].name}"`);
        }
      } else if (fins.length > 0) {
        for (let i = 0; i < Math.min(fins.length, 4); i++) {
          finPivots[i] = pivotFromMesh(fins[i]);
          console.log(`[Model] finPivots[${i}] → "${fins[i].name}" (partial)`);
        }
        if (fins.length !== 4) {
          console.warn(`[Model] Expected 4 fin nodes, found ${fins.length}. Check FBX names.`);
        }
      } else {
        console.warn("[Model] No fin nodes matched keywords — fin animation disabled. Check node names.");
      }

      if (blades.length === 0) {
        console.warn("[Model] No blade/thruster nodes matched keywords — thruster animation disabled.");
      } else {
        blades.forEach((b, i) => {
          thrusterPivots.push(pivotFromMesh(b));
          console.log(`[Model] thrusterPivots[${i}] → "${b.name}"`);
        });
      }

      const box = new THREE.Box3().setFromObject(fbx);
      const centre = box.getCenter(new THREE.Vector3());
      const size = box.getSize(new THREE.Vector3());
      const scale = 3 / Math.max(size.x, size.y, size.z);
      fbx.scale.setScalar(scale);
      fbx.position.sub(centre.multiplyScalar(scale));

      fbx.rotation.z = Math.PI / 2;
      fbx.rotation.x = Math.PI;
      fbx.rotation.y = Math.PI / 2;

      const centreGroup = new THREE.Group();
      centreGroup.add(fbx);
      const rb = new THREE.Box3().setFromObject(centreGroup);
      centreGroup.position.sub(rb.getCenter(new THREE.Vector3()));

      const imuGroup = new THREE.Group();
      imuGroup.add(centreGroup);

      centreGroup.traverse((node) => {
        if (node.isMesh && node.material) {
          const mats = Array.isArray(node.material) ? node.material : [node.material];
          mats.forEach((m) => {
            m.transparent = true;
            m.opacity = 0.55;
            m.depthWrite = false;
          });
        }
      });

      scene.add(imuGroup);
      modelWrapper = imuGroup;

      controls.target.set(0, 0, 0);
      camera.position.set(0, 0.5, 4);
      controls.update();
    },
    undefined,
    (err) => {
      loadingEl.textContent = "MODEL LOAD FAILED";
      console.error("FBX load error:", err);
    }
  );

  function resize() {
    const { width: w, height: h } = container.getBoundingClientRect();
    if (!w || !h) return;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  }
  resize();
  window.addEventListener("resize", resize);
  new ResizeObserver(resize).observe(container);

  let animFrameId = null;

  function animate() {
    animFrameId = requestAnimationFrame(animate);

    if (modelWrapper && window.subOrientation) {
      const o = window.subOrientation;
      const d2r = Math.PI / 180;
      const qPitch = new THREE.Quaternion().setFromAxisAngle(
        new THREE.Vector3(1, 0, 0), -o.pitch * d2r);
      const qRoll = new THREE.Quaternion().setFromAxisAngle(
        new THREE.Vector3(0, 0, 1), -o.roll * d2r);
      const qTarget = new THREE.Quaternion().multiplyQuaternions(qPitch, qRoll);
      modelWrapper.quaternion.slerp(qTarget, 0.08);
    }

    const servos = window.subServoAngles ?? [90, 90, 90, 90];
    for (let i = 0; i < 4; i++) {
      const pivot = finPivots[i];
      if (!pivot) continue;
      const target = ((servos[i] - 90) / 90) * MAX_FIN_RAD;
      pivot.rotation.z += (target - pivot.rotation.z) * 0.12;
    }

    const pct = window.subThrusterPct ?? 0;
    thrusterPivots.forEach((p) => {
      p.rotation.z += (pct / 100) * MAX_BLADE_RAD_PER_FRAME;
    });

    controls.update();
    renderer.render(scene, camera);
  }

  function startRender() { if (!animFrameId) animate(); }
  function stopRender() {
    if (animFrameId) { cancelAnimationFrame(animFrameId); animFrameId = null; }
  }

  document.addEventListener("visibilitychange", () => {
    if (document.hidden) stopRender();
    else startRender();
  });

  startRender();
}
