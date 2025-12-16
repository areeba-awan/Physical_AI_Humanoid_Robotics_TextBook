"use client";

import { useState } from "react";
import { useForm } from "react-hook-form";
import { zodResolver } from "@hookform/resolvers/zod";
import * as z from "zod";
import { ArrowLeft, ArrowRight, Check, Loader2 } from "lucide-react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { useAuth } from "@/hooks/use-auth";
import { cn } from "@/lib/utils";

const backgroundSchema = z.object({
  // Software Background
  softwareExperience: z.enum(['none', 'beginner', 'intermediate', 'advanced']),
  programmingLanguages: z.array(z.string()),
  rosExperience: z.boolean(),
  linuxExperience: z.enum(['none', 'basic', 'comfortable', 'expert']),

  // Hardware Background
  hardwareExperience: z.enum(['none', 'hobbyist', 'professional']),
  roboticsExperience: z.boolean(),
  previousProjects: z.string().optional(),

  // Equipment
  hasGpuWorkstation: z.boolean(),
  gpuModel: z.string().optional(),
  hasJetsonKit: z.boolean(),
  jetsonModel: z.string().optional(),
  hasRobotHardware: z.boolean(),
  robotDescription: z.string().optional(),
});

type BackgroundFormData = z.infer<typeof backgroundSchema>;

const PROGRAMMING_LANGUAGES = [
  'Python', 'C++', 'JavaScript', 'TypeScript', 'Rust', 'Go', 'Java', 'C#'
];

const JETSON_MODELS = [
  'Jetson Nano', 'Jetson TX2', 'Jetson Xavier NX', 'Jetson AGX Xavier', 'Jetson Orin'
];

const steps = [
  { id: 1, title: 'Software Background', description: 'Tell us about your programming experience' },
  { id: 2, title: 'Hardware Background', description: 'Share your hardware and robotics experience' },
  { id: 3, title: 'Equipment', description: 'What equipment do you have access to?' },
  { id: 4, title: 'Review', description: 'Review your information' },
];

export function BackgroundQuestions() {
  const [currentStep, setCurrentStep] = useState(1);
  const { updateBackground, isLoading } = useAuth();

  const {
    register,
    handleSubmit,
    watch,
    setValue,
    formState: { errors },
  } = useForm<BackgroundFormData>({
    resolver: zodResolver(backgroundSchema),
    defaultValues: {
      softwareExperience: 'beginner',
      programmingLanguages: [],
      rosExperience: false,
      linuxExperience: 'none',
      hardwareExperience: 'none',
      roboticsExperience: false,
      previousProjects: '',
      hasGpuWorkstation: false,
      gpuModel: '',
      hasJetsonKit: false,
      jetsonModel: '',
      hasRobotHardware: false,
      robotDescription: '',
    },
  });

  const formValues = watch();

  const onSubmit = async (data: BackgroundFormData) => {
    try {
      await updateBackground(data);
    } catch (error) {
      // Error handled in useAuth hook
    }
  };

  const nextStep = () => {
    if (currentStep < 4) setCurrentStep(currentStep + 1);
  };

  const prevStep = () => {
    if (currentStep > 1) setCurrentStep(currentStep - 1);
  };

  const toggleLanguage = (lang: string) => {
    const current = formValues.programmingLanguages || [];
    if (current.includes(lang)) {
      setValue('programmingLanguages', current.filter(l => l !== lang));
    } else {
      setValue('programmingLanguages', [...current, lang]);
    }
  };

  return (
    <div className="space-y-8">
      {/* Progress Indicator */}
      <div className="space-y-4">
        <div className="flex items-center justify-between text-sm">
          <span className="text-muted-foreground">Step {currentStep} of 4</span>
          <span className="text-muted-foreground">{Math.round((currentStep / 4) * 100)}% complete</span>
        </div>
        <div className="h-2 bg-muted rounded-full overflow-hidden">
          <div
            className="h-full bg-primary transition-all duration-500"
            style={{ width: `${(currentStep / 4) * 100}%` }}
          />
        </div>
        <div className="flex justify-between">
          {steps.map((step) => (
            <div
              key={step.id}
              className={cn(
                "flex flex-col items-center",
                currentStep >= step.id ? "text-primary" : "text-muted-foreground"
              )}
            >
              <div
                className={cn(
                  "w-8 h-8 rounded-full flex items-center justify-center text-sm font-medium",
                  currentStep > step.id
                    ? "bg-primary text-primary-foreground"
                    : currentStep === step.id
                    ? "border-2 border-primary"
                    : "border-2 border-muted"
                )}
              >
                {currentStep > step.id ? <Check className="h-4 w-4" /> : step.id}
              </div>
            </div>
          ))}
        </div>
      </div>

      <form onSubmit={handleSubmit(onSubmit)} className="space-y-6">
        {/* Step 1: Software Background */}
        {currentStep === 1 && (
          <div className="space-y-6 animate-fade-in">
            <div>
              <h3 className="text-lg font-semibold mb-4">Software Experience</h3>

              <div className="space-y-4">
                <div>
                  <Label className="text-base">How would you rate your programming experience?</Label>
                  <div className="grid grid-cols-2 gap-2 mt-2">
                    {[
                      { value: 'none', label: "None - I'm completely new" },
                      { value: 'beginner', label: 'Beginner - Basic understanding' },
                      { value: 'intermediate', label: 'Intermediate - Comfortable coding' },
                      { value: 'advanced', label: 'Advanced - Professional developer' },
                    ].map((option) => (
                      <label
                        key={option.value}
                        className={cn(
                          "flex items-center p-3 border rounded-lg cursor-pointer transition-colors",
                          formValues.softwareExperience === option.value
                            ? "border-primary bg-primary/5"
                            : "border-muted hover:border-primary/50"
                        )}
                      >
                        <input
                          type="radio"
                          value={option.value}
                          {...register('softwareExperience')}
                          className="sr-only"
                        />
                        <span className="text-sm">{option.label}</span>
                      </label>
                    ))}
                  </div>
                </div>

                <div>
                  <Label className="text-base">Which programming languages do you know?</Label>
                  <div className="flex flex-wrap gap-2 mt-2">
                    {PROGRAMMING_LANGUAGES.map((lang) => (
                      <button
                        key={lang}
                        type="button"
                        onClick={() => toggleLanguage(lang)}
                        className={cn(
                          "px-3 py-1.5 rounded-full text-sm font-medium transition-colors",
                          formValues.programmingLanguages?.includes(lang)
                            ? "bg-primary text-primary-foreground"
                            : "bg-muted hover:bg-muted/80"
                        )}
                      >
                        {lang}
                      </button>
                    ))}
                  </div>
                </div>

                <div>
                  <Label className="text-base">Do you have experience with ROS?</Label>
                  <div className="flex gap-4 mt-2">
                    {[
                      { value: true, label: 'Yes' },
                      { value: false, label: 'No' },
                    ].map((option) => (
                      <label
                        key={String(option.value)}
                        className={cn(
                          "flex items-center px-4 py-2 border rounded-lg cursor-pointer transition-colors",
                          formValues.rosExperience === option.value
                            ? "border-primary bg-primary/5"
                            : "border-muted hover:border-primary/50"
                        )}
                      >
                        <input
                          type="radio"
                          checked={formValues.rosExperience === option.value}
                          onChange={() => setValue('rosExperience', option.value)}
                          className="sr-only"
                        />
                        <span>{option.label}</span>
                      </label>
                    ))}
                  </div>
                </div>

                <div>
                  <Label className="text-base">Linux experience level?</Label>
                  <div className="grid grid-cols-4 gap-2 mt-2">
                    {[
                      { value: 'none', label: 'None' },
                      { value: 'basic', label: 'Basic' },
                      { value: 'comfortable', label: 'Comfortable' },
                      { value: 'expert', label: 'Expert' },
                    ].map((option) => (
                      <label
                        key={option.value}
                        className={cn(
                          "flex items-center justify-center p-2 border rounded-lg cursor-pointer transition-colors text-sm",
                          formValues.linuxExperience === option.value
                            ? "border-primary bg-primary/5"
                            : "border-muted hover:border-primary/50"
                        )}
                      >
                        <input
                          type="radio"
                          value={option.value}
                          {...register('linuxExperience')}
                          className="sr-only"
                        />
                        <span>{option.label}</span>
                      </label>
                    ))}
                  </div>
                </div>
              </div>
            </div>
          </div>
        )}

        {/* Step 2: Hardware Background */}
        {currentStep === 2 && (
          <div className="space-y-6 animate-fade-in">
            <div>
              <h3 className="text-lg font-semibold mb-4">Hardware Experience</h3>

              <div className="space-y-4">
                <div>
                  <Label className="text-base">Hardware/electronics experience?</Label>
                  <div className="grid grid-cols-3 gap-2 mt-2">
                    {[
                      { value: 'none', label: 'None' },
                      { value: 'hobbyist', label: 'Hobbyist' },
                      { value: 'professional', label: 'Professional' },
                    ].map((option) => (
                      <label
                        key={option.value}
                        className={cn(
                          "flex items-center justify-center p-3 border rounded-lg cursor-pointer transition-colors",
                          formValues.hardwareExperience === option.value
                            ? "border-primary bg-primary/5"
                            : "border-muted hover:border-primary/50"
                        )}
                      >
                        <input
                          type="radio"
                          value={option.value}
                          {...register('hardwareExperience')}
                          className="sr-only"
                        />
                        <span>{option.label}</span>
                      </label>
                    ))}
                  </div>
                </div>

                <div>
                  <Label className="text-base">Do you have robotics experience?</Label>
                  <div className="flex gap-4 mt-2">
                    {[
                      { value: true, label: 'Yes' },
                      { value: false, label: 'No' },
                    ].map((option) => (
                      <label
                        key={String(option.value)}
                        className={cn(
                          "flex items-center px-4 py-2 border rounded-lg cursor-pointer transition-colors",
                          formValues.roboticsExperience === option.value
                            ? "border-primary bg-primary/5"
                            : "border-muted hover:border-primary/50"
                        )}
                      >
                        <input
                          type="radio"
                          checked={formValues.roboticsExperience === option.value}
                          onChange={() => setValue('roboticsExperience', option.value)}
                          className="sr-only"
                        />
                        <span>{option.label}</span>
                      </label>
                    ))}
                  </div>
                </div>

                <div>
                  <Label htmlFor="previousProjects" className="text-base">
                    Describe any previous robotics or hardware projects (optional)
                  </Label>
                  <textarea
                    id="previousProjects"
                    {...register('previousProjects')}
                    className="mt-2 w-full min-h-[100px] rounded-md border border-input bg-background px-3 py-2 text-sm ring-offset-background placeholder:text-muted-foreground focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring"
                    placeholder="E.g., Built a line-following robot, Arduino projects, etc."
                  />
                </div>
              </div>
            </div>
          </div>
        )}

        {/* Step 3: Equipment */}
        {currentStep === 3 && (
          <div className="space-y-6 animate-fade-in">
            <div>
              <h3 className="text-lg font-semibold mb-4">Equipment Access</h3>

              <div className="space-y-6">
                <div className="p-4 border rounded-lg space-y-3">
                  <div className="flex items-center justify-between">
                    <Label className="text-base">Do you have a GPU workstation?</Label>
                    <div className="flex gap-2">
                      {[true, false].map((value) => (
                        <button
                          key={String(value)}
                          type="button"
                          onClick={() => setValue('hasGpuWorkstation', value)}
                          className={cn(
                            "px-3 py-1 rounded-md text-sm transition-colors",
                            formValues.hasGpuWorkstation === value
                              ? "bg-primary text-primary-foreground"
                              : "bg-muted hover:bg-muted/80"
                          )}
                        >
                          {value ? 'Yes' : 'No'}
                        </button>
                      ))}
                    </div>
                  </div>
                  {formValues.hasGpuWorkstation && (
                    <div>
                      <Label htmlFor="gpuModel">GPU Model</Label>
                      <Input
                        id="gpuModel"
                        {...register('gpuModel')}
                        placeholder="e.g., RTX 3080, RTX 4090"
                        className="mt-1"
                      />
                    </div>
                  )}
                </div>

                <div className="p-4 border rounded-lg space-y-3">
                  <div className="flex items-center justify-between">
                    <Label className="text-base">Do you have a Jetson kit?</Label>
                    <div className="flex gap-2">
                      {[true, false].map((value) => (
                        <button
                          key={String(value)}
                          type="button"
                          onClick={() => setValue('hasJetsonKit', value)}
                          className={cn(
                            "px-3 py-1 rounded-md text-sm transition-colors",
                            formValues.hasJetsonKit === value
                              ? "bg-primary text-primary-foreground"
                              : "bg-muted hover:bg-muted/80"
                          )}
                        >
                          {value ? 'Yes' : 'No'}
                        </button>
                      ))}
                    </div>
                  </div>
                  {formValues.hasJetsonKit && (
                    <div>
                      <Label>Jetson Model</Label>
                      <div className="flex flex-wrap gap-2 mt-1">
                        {JETSON_MODELS.map((model) => (
                          <button
                            key={model}
                            type="button"
                            onClick={() => setValue('jetsonModel', model)}
                            className={cn(
                              "px-3 py-1.5 rounded-full text-sm transition-colors",
                              formValues.jetsonModel === model
                                ? "bg-primary text-primary-foreground"
                                : "bg-muted hover:bg-muted/80"
                            )}
                          >
                            {model}
                          </button>
                        ))}
                      </div>
                    </div>
                  )}
                </div>

                <div className="p-4 border rounded-lg space-y-3">
                  <div className="flex items-center justify-between">
                    <Label className="text-base">Do you have robot hardware?</Label>
                    <div className="flex gap-2">
                      {[true, false].map((value) => (
                        <button
                          key={String(value)}
                          type="button"
                          onClick={() => setValue('hasRobotHardware', value)}
                          className={cn(
                            "px-3 py-1 rounded-md text-sm transition-colors",
                            formValues.hasRobotHardware === value
                              ? "bg-primary text-primary-foreground"
                              : "bg-muted hover:bg-muted/80"
                          )}
                        >
                          {value ? 'Yes' : 'No'}
                        </button>
                      ))}
                    </div>
                  </div>
                  {formValues.hasRobotHardware && (
                    <div>
                      <Label htmlFor="robotDescription">Describe your robot hardware</Label>
                      <textarea
                        id="robotDescription"
                        {...register('robotDescription')}
                        className="mt-1 w-full min-h-[80px] rounded-md border border-input bg-background px-3 py-2 text-sm"
                        placeholder="e.g., TurtleBot 3, Custom robot arm, etc."
                      />
                    </div>
                  )}
                </div>
              </div>
            </div>
          </div>
        )}

        {/* Step 4: Review */}
        {currentStep === 4 && (
          <div className="space-y-6 animate-fade-in">
            <div>
              <h3 className="text-lg font-semibold mb-4">Review Your Information</h3>

              <div className="space-y-4">
                <div className="p-4 bg-muted/50 rounded-lg space-y-3">
                  <h4 className="font-medium">Software Background</h4>
                  <div className="grid grid-cols-2 gap-2 text-sm">
                    <span className="text-muted-foreground">Experience:</span>
                    <span className="capitalize">{formValues.softwareExperience}</span>
                    <span className="text-muted-foreground">Languages:</span>
                    <span>{formValues.programmingLanguages?.join(', ') || 'None selected'}</span>
                    <span className="text-muted-foreground">ROS Experience:</span>
                    <span>{formValues.rosExperience ? 'Yes' : 'No'}</span>
                    <span className="text-muted-foreground">Linux:</span>
                    <span className="capitalize">{formValues.linuxExperience}</span>
                  </div>
                </div>

                <div className="p-4 bg-muted/50 rounded-lg space-y-3">
                  <h4 className="font-medium">Hardware Background</h4>
                  <div className="grid grid-cols-2 gap-2 text-sm">
                    <span className="text-muted-foreground">Experience:</span>
                    <span className="capitalize">{formValues.hardwareExperience}</span>
                    <span className="text-muted-foreground">Robotics:</span>
                    <span>{formValues.roboticsExperience ? 'Yes' : 'No'}</span>
                  </div>
                  {formValues.previousProjects && (
                    <p className="text-sm text-muted-foreground">
                      Projects: {formValues.previousProjects}
                    </p>
                  )}
                </div>

                <div className="p-4 bg-muted/50 rounded-lg space-y-3">
                  <h4 className="font-medium">Equipment</h4>
                  <div className="grid grid-cols-2 gap-2 text-sm">
                    <span className="text-muted-foreground">GPU Workstation:</span>
                    <span>{formValues.hasGpuWorkstation ? `Yes (${formValues.gpuModel || 'Unknown'})` : 'No'}</span>
                    <span className="text-muted-foreground">Jetson Kit:</span>
                    <span>{formValues.hasJetsonKit ? `Yes (${formValues.jetsonModel || 'Unknown'})` : 'No'}</span>
                    <span className="text-muted-foreground">Robot Hardware:</span>
                    <span>{formValues.hasRobotHardware ? 'Yes' : 'No'}</span>
                  </div>
                </div>
              </div>
            </div>
          </div>
        )}

        {/* Navigation Buttons */}
        <div className="flex items-center justify-between pt-4">
          <Button
            type="button"
            variant="outline"
            onClick={prevStep}
            disabled={currentStep === 1 || isLoading}
          >
            <ArrowLeft className="mr-2 h-4 w-4" />
            Back
          </Button>

          {currentStep < 4 ? (
            <Button type="button" onClick={nextStep}>
              Next
              <ArrowRight className="ml-2 h-4 w-4" />
            </Button>
          ) : (
            <Button type="submit" disabled={isLoading}>
              {isLoading && <Loader2 className="mr-2 h-4 w-4 animate-spin" />}
              Complete Setup
            </Button>
          )}
        </div>
      </form>
    </div>
  );
}
