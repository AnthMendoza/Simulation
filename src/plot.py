import sys
import pandas as pd
import plotly.graph_objects as go

def main():
    if not len(sys.argv) > 1:
        print("No file as argument")
        return
    data = pd.read_csv(sys.argv[1])
    df = pd.DataFrame(data).iloc[::20]
    x_data = df['timeStepVect']
    y_data = df.drop(columns=['timeStepVect'])
    fig = go.Figure()
    for column in y_data.columns:
        fig.add_trace(go.Scatter(x=x_data, y=y_data[column], mode='lines+markers', name=column))
    fig.update_layout(
        title="Multiple Data Columns vs Time",
        xaxis_title="Time(S)",
        yaxis_title="Values",
        legend_title="Variables"
    )
    fig.show()

if __name__ == "__main__":
    main()
 