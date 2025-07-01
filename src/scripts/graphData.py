import pandas as pd
import plotly.express as px


file_path = 'data.csv'  
df = pd.read_csv(file_path)



column_x = 'Time'
column_y1 = 'gimbalX'  
column_y2 = 'gimbalY'  
column_y3 = 'gimbalInputX'  
column_y4 = 'gimbalInputY'  

fig = px.line(
    df, 
    x=column_x, 
    y=[column_y1, column_y2, column_y3, column_y4],
    labels={"Time": "Time", 
            "value": "Gimbal Values"},  
    title="Time vs Gimbal Position"
)


fig.show()
