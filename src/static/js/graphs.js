
const commonAxisStyle = {
    showgrid: true,
    gridwidth: 1,
    gridcolor: '#404040',
    zerolinecolor: '#505050',
    zerolinewidth: 2,
    linecolor: '#505050',
    linewidth: 2,
    color: '#ffffff'
};

console.log(data);

const commonLayout = {
    paper_bgcolor: '#2d2d2d',
    plot_bgcolor: '#2d2d2d',
    showlegend: false,
    font: {
        family: 'Inter, Arial, sans-serif',
        size: 12,
        color: '#ffffff'
    },
    title: {
        font: {
            family: 'Inter, Arial, sans-serif',
            size: 14,
            color: '#ffffff'
        }
    },
    margin: {
        l: 40,
        r: 20,
        t: 40,
        b: 40
    }
};

// Color palette for dark theme
const colors = {
    blue: '#00bcd4',
    green: '#4caf50',
    purple: '#bb86fc',
    orange: '#ff7043',
    pink: '#ec407a',
    teal: '#009688'
};

// 1. X-Y Plot (Top Left)
const trace1 = {
    x: data.Zposition,
    y: data.gForce,
    mode: 'lines',
    line: {
        color: colors.blue,
        width: 3
    },
    name: 'Acceleration Vs Altiude'
};

Plotly.newPlot('graph1', [trace1], {
    ...commonLayout,
    title: 'Acceleration Vs Altiude',
    xaxis: {...commonAxisStyle, title: 'Altitude (meters)'},
    yaxis: {...commonAxisStyle, title: 'Acceleration (G)'}
});

// 2. X-Z Plot (Top Middle)
const trace2 = {
    x: data.Zposition,
    y: data.Velocity,
    mode: 'lines',
    line: {
        color: colors.green,
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

// 3. 3D Plot (Top Right)
const trace3 = {
    type: 'scatter3d',
    mode: 'lines',
    x: data.Xposition,
    y: data.Yposition,
    z: data.Zposition,
    line: {
        width: 6,
        color: colors.blue
    }
};

Plotly.newPlot('graph3', [trace3], {
    ...commonLayout,
    title: '3D Position',
    scene: {
        aspectmode: 'cube',
        xaxis: {
            ...commonAxisStyle,
            title: 'X (meters)',
            backgroundcolor: '#2d2d2d'
        },
        yaxis: {
            ...commonAxisStyle,
            title: 'Y (meters)',
            backgroundcolor: '#2d2d2d'
        },
        zaxis: {
            ...commonAxisStyle,
            title: 'Z (meters)',
            backgroundcolor: '#2d2d2d'
        },
        camera: {
            eye: {x: 1.5, y: 1.5, z: 1.5}
        }
    }
});

const trace4 = {
    x: data.VectorTimeStampReduced,
    y: data.gimbalAngleX,
    mode: 'lines',
    line: {
        color: colors.purple,
        width: 3
    },
    name: 'Gimbal Angle X'
};
const trace7 = {
    x: data.VectorTimeStampReduced,
    y: data.gimbalAngleY,
    mode: 'lines',
    line: {
        color: colors.red,
        width: 3
    },
    name: 'Gimbal Angle Y'
};

Plotly.newPlot('graph4', [trace4 , trace7], {
    ...commonLayout,
    title: 'Gimbal Angle',
    xaxis: {...commonAxisStyle, title: 'Time (seconds)'},
    yaxis: {...commonAxisStyle, title: 'Gimbal Angle X (Radians)'}
});

// 5. Phase Plot (Bottom Middle)
const trace5 = {
    x: data.VectorTimeStamp,
    y: data.enginePower,
    mode: 'lines',
    line: {
        color: colors.orange,
        width: 3
    },
    name: 'Phase Plot'
};

Plotly.newPlot('graph5', [trace5], {
    ...commonLayout,
    title: 'Phase Plot (dY/dt vs Y)',
    xaxis: {...commonAxisStyle, title: 'Time (seconds)'},
    yaxis: {...commonAxisStyle, title: 'Thrust / Max Thrust per Engine'}
});


const trace6 = {
    x: data.VectorTimeStampReduced,
    y: data.mass,
    mode: 'lines',
    line: {
        color: colors.teal,
        width: 3
    },
    name: 'Vehicle Mas / Time'
};

Plotly.newPlot('graph6', [trace6], {
    ...commonLayout,
    title: 'Vehicle Mas / Time',
    xaxis: {...commonAxisStyle, title: 'Time (seconds) '},
    yaxis: {...commonAxisStyle, title: 'Mass (kg)'}
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