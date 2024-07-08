from torchvision import transforms
import torch
from .cnn import cnn



class KeypointRegression:

    def __init__(self, model_src):

        self.device = (
            "cuda"
            if torch.cuda.is_available()
            else "mps"
            if torch.backends.mps.is_available()
            else "cpu"
            )
        
        print(f"KeypointRegression Device: {self.device}")

        # Load the saved model
        self.model = cnn()
        self.model.load_state_dict(torch.load(model_src, map_location=torch.device(self.device)))
        self.model.eval()  # Set model to evaluation mode


        # transform for preprocessing the images
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((80, 80)),  # input size of model
            transforms.ToTensor(),
            # Normalize: ImageNet Values for Mean and Standard Deviation in RGB
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])

    def eval(self, images):
        """
        Input: Cropped bounding box images batch
        """
        
        batch = torch.stack([self.transform(image) for image in images])
        batch = batch.to(self.device)
        print(f"batch shape: {batch.shape}")

        with torch.no_grad():  # disable gradient calculation during inference
            self.model.eval()  # set model to evaluation mode
            output = self.model(batch)

        # move output to cpu and convert to numpy array
        keypoints = output.squeeze().cpu().numpy() 

        return keypoints