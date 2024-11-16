
const t = Array.from({length: 101}, (_, i) => i * 0.1);
const x = t.map(val => val - 5);
const y = t.map(val => Math.sin(val) * 2);
const z = t.map(val => Math.cos(val) * 2);

const commonAxisStyle = {
    showgrid: true,
    gridwidth: 1,
    gridcolor: '#f0f0f0',
    zerolinecolor: '#e0e0e0',
    zerolinewidth: 2,
    linecolor: '#e0e0e0',
    linewidth: 2
};
const commonLayout = {
    paper_bgcolor: '#ffffff',
    plot_bgcolor: '#ffffff',
    showlegend: false,
    font: {
        family: 'Inter, Arial, sans-serif',
        size: 12,
        color: '#333333'
    },
    margin: {
        l: 40,
        r: 20,
        t: 40,
        b: 40
    }
};

const trace1 = {
    x: x,
    y: y,
    mode: 'lines',
    line: {
        color: '#2196F3',
        width: 3
    },
    name: 'X-Y Projection'
};
Plotly.newPlot('graph1', [trace1], {
    ...commonLayout,
    title: 'X-Y Projection',
    xaxis: {...commonAxisStyle, title: 'X (meters)'},
    yaxis: {...commonAxisStyle, title: 'Y (meters)'}
});

const trace2 = {
    x: x,
    y: z,
    mode: 'lines',
    line: {
        color: '#4CAF50',
        width: 3
    },
    name: 'X-Z Projection'
};
Plotly.newPlot('graph2', [trace2], {
    ...commonLayout,
    title: 'X-Z Projection',
    xaxis: {...commonAxisStyle, title: 'X (meters)'},
    yaxis: {...commonAxisStyle, title: 'Z (meters)'}
});

const trace3 = {
    type: 'scatter3d',
    mode: 'lines',
    x: x,
    y: y,
    z: z,
    line: {
        width: 6,
        color: '#2196F3'
    }
};
Plotly.newPlot('graph3', [trace3], {
    ...commonLayout,
    title: '3D Visualization',
    scene: {
        xaxis: {
            ...commonAxisStyle,
            title: 'X (meters)',
            backgroundcolor: '#ffffff'
        },
        yaxis: {
            ...commonAxisStyle,
            title: 'Y (meters)',
            backgroundcolor: '#ffffff'
        },
        zaxis: {
            ...commonAxisStyle,
            title: 'Z (meters)',
            backgroundcolor: '#ffffff'
        },
        camera: {
            eye: {x: 1.5, y: 1.5, z: 1.5}
        }
    }
});

const trace4 = {
    x: y,
    y: z,
    mode: 'lines',
    line: {
        color: '#9C27B0',
        width: 3
    },
    name: 'Y-Z Projection'
};
Plotly.newPlot('graph4', [trace4], {
    ...commonLayout,
    title: 'Y-Z Projection',
    xaxis: {...commonAxisStyle, title: 'Y (meters)'},
    yaxis: {...commonAxisStyle, title: 'Z (meters)'}
});

const trace5 = {
    x: y,
    y: z.map((val, i) => y[i]),
    mode: 'lines',
    line: {
        color: '#FF5722',
        width: 3
    },
    name: 'Phase Plot'
};
Plotly.newPlot('graph5', [trace5], {
    ...commonLayout,
    title: 'Phase Plot (dY/dt vs Y)',
    xaxis: {...commonAxisStyle, title: 'Y (meters)'},
    yaxis: {...commonAxisStyle, title: 'dY/dt'}
});

const trace6 = {
    x: t,
    y: y.map((val, i) => Math.sqrt(y[i]*y[i] + z[i]*z[i])),
    mode: 'lines',
    line: {
        color: '#795548',
        width: 3
    },
    name: 'Magnitude'
};
Plotly.newPlot('graph6', [trace6], {
    ...commonLayout,
    title: 'Combined Magnitude',
    xaxis: {...commonAxisStyle, title: 'Time'},
    yaxis: {...commonAxisStyle, title: 'Magnitude'}
});
const config = {
    responsive: true,
    displayModeBar: false
};
// Apply config to all plots
['graph1', 'graph2', 'graph3', 'graph4', 'graph5', 'graph6'].forEach(id => {
    Plotly.update(id, {}, {}, config);
});
// Handle window resize
window.addEventListener('resize', () => {
    ['graph1', 'graph2', 'graph3', 'graph4', 'graph5', 'graph6'].forEach(id => {
        Plotly.Plots.resize(id);
    });
});