'''
Create by - Bishwajit Kumar Poddar
'''
import os
import sys
sys.path.append(os.path.dirname(__file__))

import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.multioutput import MultiOutputRegressor
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error



class calibirator:
    def __init__(self):
        # Load data from CSV
        data = pd.read_csv(f'{os.path.dirname(__file__)}/calibiration.csv')

        # Separate input features and output variables
        input_x = data[['input_x', 'input_y']].values
        output = data[['output_x', 'output_y']].values

        # Split the data into training and testing sets
        X_train, X_test, y_train, y_test = train_test_split(input_x, output, test_size=0.1,random_state=0)

        # Create a linear regression model as the base regressor
        base_regressor = LinearRegression()

        # Create a multi-output regressor
        self.model = MultiOutputRegressor(base_regressor)

        # Fit the model to the training data
        self.model.fit(X_train, y_train)

        # Make predictions on the test data
        predictions = self.model.predict(X_test)

        # Calculate the mean squared error
        mse = mean_squared_error(y_test, predictions)

        print("Mean Squared Error:", mse)
        print('Calibiration data : ',X_train)
        print(self.model.predict(X_train))
        if mse > 0.1:
            print('Calibirated successfully')
        else:
            print('Calibiration failed')
        
    def get_location_from_pixel(self,x: int , y: int):
        real_location_x, real_location_y = self.model.predict([[x,y]])[0]
        return real_location_x, real_location_y
    
    
# Testing snippet
if __name__ == '__main__':
    object_location = calibirator()
    print(object_location.get_location_from_pixel(328,328))  