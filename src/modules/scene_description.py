from transformers import LlavaNextProcessor, LlavaNextForConditionalGeneration, AutoProcessor, BlipForConditionalGeneration
import torch
from PIL import Image


DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

class SceneDescription():
    def __init__(self):
        self.processor = AutoProcessor.from_pretrained("Salesforce/blip-image-captioning-base")
        self.model = BlipForConditionalGeneration.from_pretrained("Salesforce/blip-image-captioning-base")


    def describe(self, input):
        inputs = self.processor(images=input, return_tensors="pt")
        outputs = self.model.generate(**inputs)
        caption = self.processor.decode(outputs[0], skip_special_tokens=True)

        return caption
    

    # Here is a possible implementation of Llava-Next, but it's very very big (12gb) so rather use a smaller model
    # def __init__(self):
    #     self.processor = LlavaNextProcessor.from_pretrained("llava-hf/llava-hf/llava-1.5-7b-hf")
    #     self.model = LlavaNextForConditionalGeneration.from_pretrained("llava-hf/llava-hf/llava-1.5-7b-hf", torch_dtype=torch.float16, low_cpu_mem_usage=True) 
    #     self.model.to(DEVICE)


    # def describe(self, input):
    #     conversation = [
    #         {
    #             "role": "user",
    #             "content": [
    #                 {"type": "image"},
    #                 {"type": "text", "text": "What is shown in this image?"},
    #             ],
    #         },
    #     ]
    #     description = self.processor.apply_chat_template(conversation, add_generation_prompt=True)
    #     inputs = self.processor(description, input, return_tensors="pt").to(DEVICE)
        
    #     outputs = self.model.generate(**inputs)
    #     caption = self.processor.decode(outputs[0], skip_special_tokens=True)
        #caption = "A nice scene"

        #return caption

def main():

    captioning = SceneDescription()

    example_image = "modules/images/test.jpg"

    image = Image.open(example_image)

    caption = captioning.describe(image)

    print(caption)




if __name__ == "__main__":
    main()
