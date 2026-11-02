import numpy as np

class MinPooling:
    def __init__(self, input_samples, target_samples):
        self.input_samples = input_samples
        self.target_samples = target_samples
        self.sample_step = int(input_samples/target_samples)

    def process(self, data):
        output = np.array([np.min(data[i*self.sample_step : (i+1)*self.sample_step]) for i in range(self.target_samples)])
        return output


class IPAPRecN:
    """
    LiDAR preprocessing pipeline, that includes nonlinearity and maximizes PoS
    Link to paper: https://arxiv.org/abs/2103.11686
    In paper the algorith is learning, however (at least for now) learned parameter will become a user-defined hiperparameter
    (So technicaly it is not a 'Adaptively Parametric')
    """
    def __init__(self, beta):
        # Keep beta in range (-inf; Y_min)
        self.beta = beta

    def process(self, data):
        # Y_min = np.min(data)
        Y_min = 1.0
        output = (Y_min - self.beta) / (data - self.beta)
        return output























def main():
    processor = MinPooling(360, 36)
    data = np.random.random(size=(360,))
    output = processor.process(data)
    print(data)
    print(output)
    print(output.shape)



if __name__ == '__main__':
    main()