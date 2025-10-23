

import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd
import argparse
import sys
from pathlib import Path


def parse_multi_channel_csv(filepath):
    channels = {}
    current_channel = None
    channel_lines = []
    
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            

            if line.startswith('# '):
                if current_channel and channel_lines:
                    from io import StringIO
                    df = pd.read_csv(StringIO('\n'.join(channel_lines)))
                    channels[current_channel] = df
                    channel_lines = []
                

                current_channel = line[2:].strip()
            elif line and not line.startswith('#'):
                channel_lines.append(line)
        
        if current_channel and channel_lines:
            from io import StringIO
            df = pd.read_csv(StringIO('\n'.join(channel_lines)))
            channels[current_channel] = df
    
    return channels


def plot_scalar_channels(channels_data, title="Scalar Data", output_file=None):
    fig = go.Figure()
    
    for channel_name, df in channels_data.items():
        if 'Value' in df.columns:
            fig.add_trace(go.Scatter(
                x=df['Timestamp'],
                y=df['Value'],
                mode='lines',
                name=channel_name,
                line=dict(width=2),
                hovertemplate='<b>%{fullData.name}</b><br>' +
                              'Time: %{x:.3f}s<br>' +
                              'Value: %{y:.4f}<extra></extra>'
            ))
    
    fig.update_layout(
        title=title,
        xaxis_title="Time (seconds)",
        yaxis_title="Value",
        hovermode='x unified',
        template='plotly_white',
        legend=dict(
            yanchor="top",
            y=0.99,
            xanchor="left",
            x=0.01
        ),
        height=600
    )
    
    if output_file:
        fig.write_html(output_file)
        print(f"Saved scalar plot to {output_file}")
    else:
        fig.show()
    
    return fig


def plot_vector3_channels(channels_data, title="3D Vector Data", output_file=None):
    fig = make_subplots(
        rows=3, cols=1,
        subplot_titles=('X Component', 'Y Component', 'Z Component'),
        shared_xaxes=True,
        vertical_spacing=0.08
    )
    
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', 
              '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    
    for idx, (channel_name, df) in enumerate(channels_data.items()):
        if all(col in df.columns for col in ['X', 'Y', 'Z']):
            color = colors[idx % len(colors)]
            
            fig.add_trace(go.Scatter(
                x=df['Timestamp'], y=df['X'],
                mode='lines',
                name=f"{channel_name}",
                line=dict(color=color, width=2),
                legendgroup=channel_name,
                hovertemplate=f'<b>{channel_name}</b><br>Time: %{{x:.3f}}s<br>X: %{{y:.4f}}<extra></extra>'
            ), row=1, col=1)
            

            fig.add_trace(go.Scatter(
                x=df['Timestamp'], y=df['Y'],
                mode='lines',
                name=f"{channel_name}",
                line=dict(color=color, width=2),
                legendgroup=channel_name,
                showlegend=False,
                hovertemplate=f'<b>{channel_name}</b><br>Time: %{{x:.3f}}s<br>Y: %{{y:.4f}}<extra></extra>'
            ), row=2, col=1)
            
            fig.add_trace(go.Scatter(
                x=df['Timestamp'], y=df['Z'],
                mode='lines',
                name=f"{channel_name}",
                line=dict(color=color, width=2),
                legendgroup=channel_name,
                showlegend=False,
                hovertemplate=f'<b>{channel_name}</b><br>Time: %{{x:.3f}}s<br>Z: %{{y:.4f}}<extra></extra>'
            ), row=3, col=1)
    
    fig.update_xaxes(title_text="Time (seconds)", row=3, col=1)
    fig.update_yaxes(title_text="X Value", row=1, col=1)
    fig.update_yaxes(title_text="Y Value", row=2, col=1)
    fig.update_yaxes(title_text="Z Value", row=3, col=1)
    
    fig.update_layout(
        title_text=title,
        hovermode='x unified',
        template='plotly_white',
        height=900,
        showlegend=True
    )
    
    if output_file:
        fig.write_html(output_file)
        print(f"Saved vector plot to {output_file}")
    else:
        fig.show()
    
    return fig


def plot_3d_trajectory(channels_data, title="3D Trajectory", output_file=None):
    fig = go.Figure()
    
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
    
    for idx, (channel_name, df) in enumerate(channels_data.items()):
        if all(col in df.columns for col in ['X', 'Y', 'Z']):
            color = colors[idx % len(colors)]
            
            fig.add_trace(go.Scatter3d(
                x=df['X'], y=df['Y'], z=df['Z'],
                mode='lines',
                name=channel_name,
                line=dict(color=color, width=4),
                hovertemplate=f'<b>{channel_name}</b><br>' +
                              'X: %{x:.3f}<br>' +
                              'Y: %{y:.3f}<br>' +
                              'Z: %{z:.3f}<extra></extra>'
            ))
            
            fig.add_trace(go.Scatter3d(
                x=[df['X'].iloc[0]], 
                y=[df['Y'].iloc[0]], 
                z=[df['Z'].iloc[0]],
                mode='markers',
                name=f"{channel_name} (start)",
                marker=dict(size=8, color='green', symbol='circle'),
                showlegend=False
            ))
            
            fig.add_trace(go.Scatter3d(
                x=[df['X'].iloc[-1]], 
                y=[df['Y'].iloc[-1]], 
                z=[df['Z'].iloc[-1]],
                mode='markers',
                name=f"{channel_name} (end)",
                marker=dict(size=8, color='red', symbol='diamond'),
                showlegend=False
            ))
    
    fig.update_layout(
        title=title,
        scene=dict(
            xaxis_title='X',
            yaxis_title='Y',
            zaxis_title='Z',
            aspectmode='data'
        ),
        template='plotly_white',
        height=800
    )
    
    if output_file:
        fig.write_html(output_file)
        print(f"Saved 3D trajectory plot to {output_file}")
    else:
        fig.show()
    
    return fig


def main():
    parser = argparse.ArgumentParser(
        description='Plot multi-channel CSV data from drone simulation logs'
    )
    parser.add_argument('csv_file', type=str, help='Path to the CSV file')
    parser.add_argument('--output', '-o', type=str, default=None,
                       help='Output HTML file (default: show interactive plot)')
    parser.add_argument('--type', '-t', type=str, 
                       choices=['auto', 'scalar', 'vector', '3d', 'all'],
                       default='auto',
                       help='Plot type (default: auto-detect)')
    
    args = parser.parse_args()
    

    if not Path(args.csv_file).exists():
        print(f"Error: File '{args.csv_file}' not found")
        sys.exit(1)
    
    print(f"Reading {args.csv_file}...")
    channels = parse_multi_channel_csv(args.csv_file)
    
    if not channels:
        print("Error: No channels found in CSV file")
        sys.exit(1)
    
    print(f"Found {len(channels)} channels:")
    for name, df in channels.items():
        print(f"  - {name}: {len(df)} samples, columns: {list(df.columns)}")
    
    scalar_channels = {name: df for name, df in channels.items() 
                      if 'Value' in df.columns}
    vector_channels = {name: df for name, df in channels.items() 
                      if all(col in df.columns for col in ['X', 'Y', 'Z'])}
    
    if args.type == 'auto' or args.type == 'all':
        if scalar_channels:
            output = f"{Path(args.csv_file).stem}_scalar.html" if args.output else None
            plot_scalar_channels(scalar_channels, 
                               title="Scalar Channels", 
                               output_file=output)
        
        if vector_channels:
            output = f"{Path(args.csv_file).stem}_vector.html" if args.output else None
            plot_vector3_channels(vector_channels, 
                                title="Vector Channels (X, Y, Z)", 
                                output_file=output)
            
            output = f"{Path(args.csv_file).stem}_3d.html" if args.output else None
            plot_3d_trajectory(vector_channels,
                             title="3D Trajectories",
                             output_file=output)
    
    elif args.type == 'scalar' and scalar_channels:
        plot_scalar_channels(scalar_channels, output_file=args.output)
    
    elif args.type == 'vector' and vector_channels:
        plot_vector3_channels(vector_channels, output_file=args.output)
    
    elif args.type == '3d' and vector_channels:
        plot_3d_trajectory(vector_channels, output_file=args.output)
    
    else:
        print(f"Error: No channels found for plot type '{args.type}'")
        sys.exit(1)
    
    print("Done!")


if __name__ == "__main__":
    main()