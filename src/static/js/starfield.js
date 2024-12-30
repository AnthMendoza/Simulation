function createStars() {
    const body = document.body;
    for (let i = 0; i < 50; i++) {
        const star = document.createElement('div');
        star.className = 'star';

        const x = Math.random() * 100;
        const y = Math.random() * 100;
        
        const size = Math.random() * 3 + 1;
        
        const delay = Math.random() * 40;
        
        star.style.left = `${x}%`;
        star.style.top = `${y}%`;
        star.style.width = `${size}px`;
        star.style.height = `${size}px`;
        star.style.animationDelay = `${delay}s`;
        
        body.appendChild(star);
    }
}

createStars();