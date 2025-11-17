import plotly.graph_objects as go

def plot(data):
    if not data:
        print("No data to plot.")
        return

    # Each row = [x, y1, y2, y3, ...]
    x_vals = [row[0] for row in data]
    num_cols = len(data[0])

    fig = go.Figure()

    # Plot each Y column after the first
    for col in range(1, num_cols):
        y_vals = [row[col] for row in data]
        fig.add_trace(go.Scatter(
            x=x_vals,
            y=y_vals,
            mode='lines',
            name=f'Y{col}'
        ))

    fig.update_layout(
        title='Simulation Plot',
        xaxis_title='X (time)',
        yaxis_title='Values',
        template='plotly_dark'
    )

    fig.show()
