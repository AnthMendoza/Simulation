import * as THREE from "three";

import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

const vertexShader = document.getElementById("vertex-shader").textContent;
const fragShader = document.getElementById("fragment-shader").textContent;

const scene = new THREE.Scene();
const width = window.innerWidth;
const height = window.innerHeight;
const camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
camera.position.z = 3;

const renderer = new THREE.WebGLRenderer();
renderer.setSize(width, height);
renderer.pixelRatio = window.devicePixelRatio ?? renderer.pixelRatio;
renderer.autoClear = false;

document.body.appendChild(renderer.domElement);

const getMaterial = (color) =>
    new THREE.ShaderMaterial({
        fragmentShader: fragShader,
        vertexShader: vertexShader,
        glslVersion: THREE.GLSL3,
        uniforms: {
            uAnimationProgress: { value: (Date.now() / 2000) % 1 },
            uColor: { value: color },
        },
        transparent: true,
    });

const material1 = getMaterial([1, 0, 0]);
const material2 = getMaterial([1, 1, 0]);

const loader = new GLTFLoader();
loader.load(
    "../static/3d/fire-template.glb",
    function (gltf) {
        const { geometry } = gltf.scene.children[0];
        const mesh1 = new THREE.Mesh(geometry, material1);
        mesh1.position.y = -0.55;

        const mesh2 = new THREE.Mesh(geometry, material2);
        mesh2.scale.set(0.9, 0.9, 0.9);
        mesh2.position.y = -0.55;

        scene.add(mesh1);
        scene.add(mesh2);

        function render() {
            material1.uniforms.uAnimationProgress.value = (Date.now() / 2000) % 1;
            material2.uniforms.uAnimationProgress.value =(Date.now() / 2000 + 0.2) % 1;

            material1.side = THREE.BackSide;
            material2.side = THREE.BackSide;
            mesh1.renderOrder = 0;
            mesh2.renderOrder = 1;
            renderer.render(scene, camera);

            material2.side = THREE.FrontSide;
            material1.side = THREE.FrontSide;
            mesh1.renderOrder = 1;
            mesh2.renderOrder = 0;
            renderer.render(scene, camera);
            window.requestAnimationFrame(render);
        }

        render();
    },
    undefined,
    console.error
);
