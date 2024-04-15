import torch.optim as optim
import os
import torch
from torch import nn
from torch.utils.data import DataLoader
from pose_estimation import cnn
import json
from PIL import Image
from torch.utils.data import Dataset
import torchvision.transforms as transforms

# Assuming your data is in a directory called 'data_dir' and is organized into 'train' and 'val' directories
data_dir = 'data'

class CustomDataset(Dataset):
    def __init__(self, data_dir, annotations_file, transform=None):
        self.data_dir = data_dir
        self.annotations_file = annotations_file
        self.transform = transform
        self.annotations = self.load_annotations()

    def __len__(self):
        return len(self.annotations)

    def __getitem__(self, idx):
        img_name = self.annotations[idx]['img']
        img_path = os.path.join(self.data_dir, img_name)
        image = Image.open(img_path).convert('RGB')
        annotation = self.annotations[idx]['kp-1']

        if self.transform:
            image = self.transform(image)

        return image, annotation

    def load_annotations(self):
        with open(self.annotations_file, 'r') as f:
            annotations = json.load(f)
        return annotations

# Define transforms
data_transforms = {
    'train': transforms.Compose([
        transforms.RandomResizedCrop(80),
        transforms.RandomHorizontalFlip(),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ]),
    'val': transforms.Compose([
        transforms.Resize(80),
        transforms.CenterCrop(80),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ]),
}

# Load datasets with annotations
custom_datasets = {x: CustomDataset(data_dir=os.path.join(data_dir, x),
                                    annotations_file=os.path.join(data_dir, 'ann.json'),
                                    transform=data_transforms[x]) for x in ['train', 'val']}

# Create data loaders
dataloaders = {x: DataLoader(custom_datasets[x], batch_size=4, shuffle=True, num_workers=4) for x in ['train', 'val']}


device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu") # or mps

# Initialize the CNN
model = cnn()
model = model.to(device)

# Define a Loss function and optimizer
criterion = nn.CrossEntropyLoss()
optimizer = optim.SGD(model.parameters(), lr=0.001, momentum=0.9)

# Train the network
for epoch in range(2):  # loop over the dataset multiple times
    running_loss = 0.0
    for i, data in enumerate(dataloaders['train'], 0):
        # get the inputs; data is a list of [inputs, labels]
        inputs, labels = data[0].to(device), data[1].to(device)

        # zero the parameter gradients
        optimizer.zero_grad()

        # forward + backward + optimize
        outputs = model(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()

        # print statistics
        running_loss += loss.item()
        if i % 2000 == 1999:    # print every 2000 mini-batches
            print('[%d, %5d] loss: %.3f' % (epoch + 1, i + 1, running_loss / 2000))
            running_loss = 0.0

print('Finished Training')

