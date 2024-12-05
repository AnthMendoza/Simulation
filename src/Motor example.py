import numpy as np
import plotly.graph_objs as go

def create_function_graph(func, start, end, step_size=0.1, title="Function Graph"):
    x_array = np.arange(start, end + step_size, step_size)
    
  
    y_array = np.zeros_like(x_array)
    
    current_rpm = 0
    
    for i, x in enumerate(x_array):

        current_rpm = func(current_rpm, step_size)
        y_array[i] = current_rpm

    fig = go.Figure()
    
    fig.add_trace(go.Scatter(
        x=x_array, 
        y=y_array, 
        name='RPM Simulation'
    ))
    
    # Customize layout
    fig.update_layout(
        title=title,
        xaxis_title='Time (s)',
        yaxis_title='RPM',
        template='plotly_white'
    )
    
    # Show the plot
    fig.show()
    
    return x_array, y_array

def rpm_simulation(current_rpm, step_size):

    resistance = 0.5  # nm
    max_torque = 30  # nm
    max_rpm = 5000  # maximum RPM
    moment_of_inertia = .001  # kg*m^2
    
    torque = ((1 - current_rpm/max_rpm)) * max_torque - resistance
    if (torque > max_torque):
        torque = max_torque
    rpm = current_rpm + torque / moment_of_inertia * step_size
    
    
    rpm = max(0, rpm)
    print(current_rpm)
    
    return rpm

if __name__ == "__main__":
    step_size = 0.1
    x_values, y_values = create_function_graph(
        func=rpm_simulation, 
        start=0, 
        end=1, 
        step_size=step_size, 
        title="RPM Simulation"
    )
    
    print("X values:", x_values[:10])
    print("Y values:", y_values[:10])