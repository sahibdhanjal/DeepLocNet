import scipy.io as sci
from pdb import set_trace as bp
from keras.layers import Input, Dense, Flatten, Reshape
from keras.models import Model, Sequential
from keras import regularizers
import numpy as np

class autoEncoder:
    def __init__(self, xTrain, xTest, shape = [10, 20, 50, 100], epochs = 1000, batchSize = 16384, addReg = False, printSummary = False):
        self.shape = shape
        self.epochs = epochs
        self.batchSize = batchSize
        self.addReg = addReg
        self.autoencoder = Sequential()
        self.encoder = None
        self.xTrain = xTrain
        self.xTest = xTest
        self.input_dim = xTrain.shape[1]
        self.initialize()

    def initialize(self, **kwargs):
        if not self.addReg:
            # Encoder Layers
            self.autoencoder.add(Dense(self.shape[0], input_shape=(2,), activation='relu'))
            self.autoencoder.add(Dense(self.shape[1], activation='relu'))
            self.autoencoder.add(Dense(self.shape[2], activation='relu'))
            self.autoencoder.add(Dense(self.shape[3], activation='relu'))
            # Decoder Layers
            self.autoencoder.add(Dense(self.shape[2], activation='relu'))
            self.autoencoder.add(Dense(self.shape[1], activation='relu'))
            self.autoencoder.add(Dense(self.shape[0], activation='relu'))
            self.autoencoder.add(Dense(2, activation='sigmoid'))

        else:
            # Encoder Layers
            self.autoencoder.add(Dense(self.shape[0], input_shape=(2,), activation='relu',activity_regularizer=regularizers.l2(10e-5)))
            self.autoencoder.add(Dense(self.shape[1], activation='relu',activity_regularizer=regularizers.l2(10e-5)))
            self.autoencoder.add(Dense(self.shape[2], activation='relu',activity_regularizer=regularizers.l2(10e-5)))
            self.autoencoder.add(Dense(self.shape[3], activation='relu',activity_regularizer=regularizers.l2(10e-5)))
            # Decoder Layers
            self.autoencoder.add(Dense(self.shape[2], activation='relu'))
            self.autoencoder.add(Dense(self.shape[1], activation='relu'))
            self.autoencoder.add(Dense(self.shape[0], activation='relu'))
            self.autoencoder.add(Dense(2, activation='sigmoid'))

        if 'printSummary' in kwargs.keys():
            self.autoencoder.summary()

    def Encoder(self, **kwargs):
        input_data = Input(shape=(2,))
        encoder_layer1 = self.autoencoder.layers[0]
        encoder_layer2 = self.autoencoder.layers[1]
        encoder_layer3 = self.autoencoder.layers[2]
        encoder_layer4 = self.autoencoder.layers[3]
        self.encoder = Model(input_data, encoder_layer4(encoder_layer3(encoder_layer2(encoder_layer1(input_data)))))

        if 'printSummary' in kwargs.keys():
            self.encoder.summary()

    def train(self, x_train, x_test):
        self.Encoder()
        self.autoencoder.compile(optimizer = 'adam', loss = 'binary_crossentropy')
        self.autoencoder.fit(x_train, x_train, epochs = self.epochs, batch_size = self.batchSize, validation_data = (x_test, x_test))

    def save(self):
        self.autoencoder.save('stacked.h5')
