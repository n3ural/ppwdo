from django.shortcuts import render, HttpResponse


# Create your views here.
def home(request):
    return render(request, 'index.html', {"test": {'0': 'a', '1': 'b'}})
