const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer();
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

// Particle properties
const particleCount = 100000;
const particleGeometry = new THREE.BufferGeometry();
const particleMaterial = new THREE.PointsMaterial({ color: 0xaa4203, size: 0.05 });
const particlePositions = new Float32Array(particleCount * 3);
const particlesData = [];
const maxLifetime = 1;

// Create origin that will move
const origin = new THREE.Vector3(-2, -3, 0);
const range = 1.4;

// Parameters for origin movement
const radius = 3; // Radius of circular motion
const verticalAmplitude = 1; // How high/low it bobs
const horizontalSpeed = 0.1; // Speed of circular motion
const verticalSpeed = 0.1; // Speed of up/down motion
let time = 0; // Time tracker for movement

// Initialize particle data
for (let i = 0; i < particleCount; i++) {
    const particle = {
        position: new THREE.Vector3(
            origin.x + (Math.random() - 0.5) * range,
            origin.y + (Math.random() - 0.5) * range,
            origin.z + (Math.random() - 0.5) * range
        ),
        direction: new THREE.Vector3(1, 1, 3),
        lifetime: Math.random() * maxLifetime
    };
    particlesData.push(particle);

    particlePositions[i * 3] = particle.position.x;
    particlePositions[i * 3 + 1] = particle.position.y;
    particlePositions[i * 3 + 2] = particle.position.z;
}

particleGeometry.setAttribute('position', new THREE.BufferAttribute(particlePositions, 3));
const particles = new THREE.Points(particleGeometry, particleMaterial);
scene.add(particles);

camera.position.z = 15; // Moved camera back to see the full motion
camera.position.y = 5;  // Slightly elevated view
camera.lookAt(0, 0, 0);

const speed = 0.05;



function updateFire() {
    // Update origin position first
    origin.x = Math.cos(time * horizontalSpeed) * radius;
    origin.z = Math.sin(time * horizontalSpeed) * radius;
    origin.y = Math.sin(time * verticalSpeed) * verticalAmplitude - 3;
    time += 1;

    const positions = particles.geometry.attributes.position.array;

    for (let i = 0; i < particleCount; i++) {
        const particle = particlesData[i];

        particle.lifetime -= 0.016;

        if (particle.lifetime <= 0) {
            // Reset particle position relative to new origin position
            particle.position.set(
                origin.x + (Math.random() - 0.5) * range,
                origin.y + (Math.random() - 0.5) * range,
                origin.z + (Math.random() - 0.5) * range
            );
            particle.lifetime = maxLifetime;
        }

        particle.position.addScaledVector(particle.direction, speed);

        positions[i * 3] = particle.position.x;
        positions[i * 3 + 1] = particle.position.y;
        positions[i * 3 + 2] = particle.position.z;
    }

    particles.geometry.attributes.position.needsUpdate = true;
}

function animate() {
    requestAnimationFrame(animate);
    updateFire();
    renderer.render(scene, camera);
}

animate();